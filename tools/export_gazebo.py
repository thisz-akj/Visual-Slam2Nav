"""Export a fused SLAM mesh as a Gazebo (Harmonic) world.

    1. clean      drop small disconnected fragments
    2. level      find the floor plane, rotate it to z = 0 (gravity up = +z)
    3. square     rotate about z so the dominant walls align with x / y
    4. simplify   quadric decimation: visual mesh + lighter collision mesh
    5. texture    bake vertex colours into a UV texture (Gazebo ignores vertex colours)
    6. package    gazebo/models/<name>/ (model.sdf, model.config, meshes/) + gazebo/worlds/<name>.sdf

Usage:
    .venv/bin/python tools/export_gazebo.py --run runs/apt0 --name apt0
    GZ_SIM_RESOURCE_PATH=$PWD/gazebo/models gz sim gazebo/worlds/apt0.sdf
"""
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parents[1]


def clean(mesh, min_tris):
    tri_cluster, n_tris, _ = mesh.cluster_connected_triangles()
    tri_cluster, n_tris = np.asarray(tri_cluster), np.asarray(n_tris)
    mesh.remove_triangles_by_mask(n_tris[tri_cluster] < min_tris)
    mesh.remove_unreferenced_vertices()
    print(f"clean: kept {len(mesh.triangles):,} triangles ({(n_tris >= min_tris).sum()} of {len(n_tris)} pieces)")
    return mesh


def find_floor(mesh, cams, cam_rot):
    """Lowest large horizontal plane below the cameras.

    'Down' is estimated as the camera y axis averaged over the whole run: a handheld
    scanner is held roughly upright on average, while any single frame (e.g. the first)
    may be pitched a lot.
    """
    down = cam_rot[:, :, 1].mean(0)
    down /= np.linalg.norm(down)
    pcd = mesh.sample_points_uniformly(400_000)
    cam_c = np.median(cams, axis=0)
    cands = []
    rest = pcd
    for _ in range(12):
        if len(rest.points) < 5000:
            break
        (a, b, c, d), inl = rest.segment_plane(0.03, 3, 2000)
        n = np.array([a, b, c])
        h = n @ cam_c + d                        # signed height of cameras above the plane
        if h < 0:
            n, d, h = -n, -d, -h                 # make n point from the plane toward the cameras
        if n @ down < -0.9 and 0.3 < h < 2.5:   # horizontal, below the cameras
            cands.append((h, len(inl), n, d))
        rest = rest.select_by_index(inl, invert=True)
    if not cands:
        raise RuntimeError("no floor plane found")
    lowest = max(c[0] for c in cands)
    big = max(c[1] for c in cands)
    # floor = lowest plane with real support; tables/sofas are parallel but higher
    h, k, n, d = max((c for c in cands if c[1] > 0.2 * big and c[0] > lowest - 0.15), key=lambda c: c[1])
    print(f"gravity estimate {np.round(down, 3)}; floor normal {np.round(n, 3)}, "
          f"cameras {h:.2f} m above it, {k:,} support points ({len(cands)} horizontal planes)")
    return n, d


def rot_a_to_b(a, b):
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), a @ b
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1 + c)


def wall_yaw(mesh):
    """Dominant wall direction from near-horizontal face normals (mod 90 deg)."""
    mesh.compute_triangle_normals()
    N = np.asarray(mesh.triangle_normals)
    N = N[np.abs(N[:, 2]) < 0.2]
    ang = np.mod(np.arctan2(N[:, 1], N[:, 0]), np.pi / 2)
    hist, edges = np.histogram(ang, bins=90, range=(0, np.pi / 2))
    k = np.argmax(np.convolve(np.r_[hist[-2:], hist, hist[:2]], np.ones(5), "valid"))
    return (edges[k] + edges[k + 1]) / 2


def bake_texture(mesh, tex_size):
    """UV-unwrap and bake vertex colours into an albedo texture (tensor API)."""
    # UVAtlas needs a manifold mesh; decimation can leave duplicate / non-manifold edges
    n0 = len(mesh.triangles)
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_non_manifold_edges()
    mesh.remove_unreferenced_vertices()
    print(f"  manifold repair: removed {n0 - len(mesh.triangles):,} triangles")
    t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    t.compute_uvatlas(size=tex_size)
    tex = t.bake_vertex_attr_textures(tex_size, {"colors"})
    img = (np.clip(tex["colors"].numpy(), 0, 1) * 255).astype(np.uint8)
    out = t.to_legacy()
    out.textures = [o3d.geometry.Image(np.ascontiguousarray(img[::-1]))]   # OBJ uv origin is bottom-left
    out.triangle_material_ids = o3d.utility.IntVector(np.zeros(len(out.triangles), np.int32))
    out.vertex_colors = o3d.utility.Vector3dVector()
    return out


def bake_texture_per_triangle(mesh, source, tex_size, chunk_rows=256):
    """Per-triangle atlas: every triangle gets its own square cell in the texture.

    No UV unwrapping (so no manifold requirement, and fast for millions of triangles).
    Each texel is coloured from the nearest vertex of `source` - the full-resolution
    mesh - so the texture keeps detail that decimation removed from `mesh`.
    """
    from scipy.spatial import cKDTree
    V, T = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    # Spatially coherent atlas: order triangles along a 3D Morton (Z-order) curve, so cells that are
    # neighbours in the texture are neighbours in space. Texture filtering / mipmaps then blend similar
    # colours instead of unrelated ones (otherwise: dark seams and confetti at a distance).
    c = V[T].mean(1)
    q = ((c - c.min(0)) / (np.ptp(c, 0).max() + 1e-9) * 1023).astype(np.uint64)
    code = np.zeros(len(T), np.uint64)
    for bit in range(10):
        for axis in range(3):
            code |= ((q[:, axis] >> np.uint64(bit)) & np.uint64(1)) << np.uint64(3 * bit + axis)
    T = T[np.argsort(code, kind="stable")]
    n = len(T)
    grid = int(np.ceil(np.sqrt(n)))
    s = tex_size // grid                                  # cell size in pixels
    if s < 4:
        raise ValueError(f"texture too small: {tex_size}px for {n:,} triangles")
    m, L = 1.0, s - 2.0                                    # 1 px padding inside each cell
    W = H = tex_size
    k = np.arange(n)
    cx, cy = (k % grid) * s, (k // grid) * s
    # triangle corners in pixel coords: P0=(m,m), P1=(m+L,m), P2=(m,m+L) inside the cell
    px = np.stack([cx + m, cx + m + L, cx + m], 1)
    py = np.stack([cy + m, cy + m, cy + m + L], 1)
    # Gazebo (ogre2 / assimp) samples OBJ textures with v measured from the TOP row
    # (verified by rendering), the opposite of the usual OBJ convention.
    uvs = np.stack([px / W, py / H], 2).reshape(-1, 2)

    src_pts = np.asarray(source.vertices)
    src_col = np.asarray(source.vertex_colors)
    tree = cKDTree(src_pts)
    img = np.zeros((H, W, 3), np.uint8)
    xs = np.arange(W)
    for y0 in range(0, grid * s, chunk_rows):
        ys = np.arange(y0, min(y0 + chunk_rows, grid * s))
        X, Y = np.meshgrid(xs[: grid * s], ys)
        tri = (Y // s) * grid + (X // s)
        ok = tri < n
        X, Y, tri = X[ok], Y[ok], tri[ok]
        u = ((X % s) + 0.5 - m) / L
        v = ((Y % s) + 0.5 - m) / L
        b = np.clip(np.stack([1 - u - v, u, v], 1), 0, None)     # clamp padding texels onto the triangle
        b /= b.sum(1, keepdims=True)
        P = np.einsum("nk,nkd->nd", b, V[T[tri]])
        _, nn = tree.query(P, workers=-1)
        img[Y, X] = (np.clip(src_col[nn], 0, 1) * 255).astype(np.uint8)
    out = o3d.geometry.TriangleMesh(mesh.vertices, o3d.utility.Vector3iVector(T))
    out.triangle_uvs = o3d.utility.Vector2dVector(uvs)
    out.textures = [o3d.geometry.Image(img)]
    out.triangle_material_ids = o3d.utility.IntVector(np.zeros(n, np.int32))
    print(f"  per-triangle atlas: {grid}x{grid} cells of {s}px, colours from {len(src_pts):,} source vertices")
    return out


MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="map">
      <visual name="visual">
        <geometry><mesh><uri>meshes/{name}_visual.obj</uri></mesh></geometry>
      </visual>
      <collision name="collision">
        <geometry><mesh><uri>meshes/{name}_collision.stl</uri></mesh></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Indoor map reconstructed with ORB-SLAM3 RGB-D + TSDF fusion ({src}).</description>
</model>
"""

WORLD_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{name}_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.7 0.7 0.75 1</background>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <!-- flat physics floor at z=0 (scanned floor is visual only) -->
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material></visual>
      </link>
    </model>

    <include>
      <uri>model://{name}</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, type=Path)
    ap.add_argument("--name", required=True)
    ap.add_argument("--visual-tris", type=int, default=300_000)
    ap.add_argument("--collision-tris", type=int, default=60_000)
    ap.add_argument("--tex-size", type=int, default=4096)
    ap.add_argument("--texture", choices=["uvatlas", "per-triangle"], default="uvatlas",
                    help="uvatlas: compact unwrap (slow, needs manifold mesh, < ~500k tris); "
                         "per-triangle: one cell per triangle, colours from the full-res mesh (fast, any size)")
    ap.add_argument("--align-like", help="reuse T_slam_to_gazebo.txt of this existing model "
                                         "(same SLAM run, e.g. a re-fused mesh) instead of re-estimating it")
    args = ap.parse_args()

    mesh = o3d.io.read_triangle_mesh(str(args.run / "dense_mesh.ply"))
    traj = np.loadtxt(args.run / "CameraTrajectory.txt")
    cams = traj[:, 1:4]
    from scipy.spatial.transform import Rotation
    cam_rot = Rotation.from_quat(traj[:, 4:8]).as_matrix()
    print(f"loaded {len(mesh.triangles):,} triangles")
    mesh = clean(mesh, min_tris=2000)

    if args.align_like:
        T_gz = np.loadtxt(ROOT / "gazebo" / "models" / args.align_like / "T_slam_to_gazebo.txt")
        mesh.transform(T_gz)
        lo, hi = mesh.get_min_bound(), mesh.get_max_bound()
        print(f"alignment reused from {args.align_like}; footprint {hi[0]-lo[0]:.2f} x {hi[1]-lo[1]:.2f} m, "
              f"height {lo[2]:.2f} .. {hi[2]:.2f} m")
    else:
        # level: floor normal -> +z, floor at z = 0
        n, d = find_floor(mesh, cams, cam_rot)
        R = rot_a_to_b(n, np.array([0.0, 0.0, 1.0]))
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = R @ (d * n)                        # floor point p0 = -d*n must map to 0: t = -R p0
        mesh.transform(T)

        # square: align walls with x / y, then centre the footprint at the origin
        yaw = wall_yaw(mesh)
        Rz = np.eye(4)
        Rz[:2, :2] = [[np.cos(-yaw), -np.sin(-yaw)], [np.sin(-yaw), np.cos(-yaw)]]
        mesh.transform(Rz)
        lo, hi = mesh.get_min_bound(), mesh.get_max_bound()
        C = np.eye(4)
        C[:2, 3] = -(lo[:2] + hi[:2]) / 2
        mesh.transform(C)
        T_gz = C @ Rz @ T                              # SLAM world -> Gazebo world
        lo, hi = mesh.get_min_bound(), mesh.get_max_bound()
        print(f"walls rotated by {np.degrees(yaw):.1f} deg; footprint {hi[0]-lo[0]:.2f} x {hi[1]-lo[1]:.2f} m, "
              f"height {lo[2]:.2f} .. {hi[2]:.2f} m")

    model = ROOT / "gazebo" / "models" / args.name
    (model / "meshes").mkdir(parents=True, exist_ok=True)
    worlds = ROOT / "gazebo" / "worlds"
    worlds.mkdir(parents=True, exist_ok=True)

    col = mesh.simplify_quadric_decimation(args.collision_tris)
    # flat floor for physics: drop scanned floor triangles, the world's ground plane at z=0 replaces them
    # (a lumpy reconstructed floor makes wheeled robots stall)
    col.compute_triangle_normals()
    zc = np.asarray(col.vertices)[np.asarray(col.triangles)].mean(1)[:, 2]
    col.remove_triangles_by_mask((zc < 0.06) & (np.abs(np.asarray(col.triangle_normals)[:, 2]) > 0.7))
    col.remove_unreferenced_vertices()
    col.compute_triangle_normals()
    o3d.io.write_triangle_mesh(str(model / "meshes" / f"{args.name}_collision.stl"), col)
    print(f"collision mesh: {len(col.triangles):,} triangles")

    vis = mesh.simplify_quadric_decimation(args.visual_tris)
    print(f"visual mesh: {len(vis.triangles):,} triangles, baking {args.tex_size}px texture (takes a few minutes)...")
    if args.texture == "per-triangle":
        vis = bake_texture_per_triangle(vis, mesh, args.tex_size)
    else:
        vis = bake_texture(vis, args.tex_size)
    vis.compute_vertex_normals()
    o3d.io.write_triangle_mesh(str(model / "meshes" / f"{args.name}_visual.obj"), vis)

    (model / "model.sdf").write_text(MODEL_SDF.format(name=args.name))
    (model / "model.config").write_text(MODEL_CONFIG.format(name=args.name, src=args.run))
    (worlds / f"{args.name}.sdf").write_text(WORLD_SDF.format(name=args.name))

    # camera path in Gazebo coordinates: handy for spawning robots / checking alignment
    P = (T_gz[:3, :3] @ cams.T).T + T_gz[:3, 3]
    np.savetxt(model / "camera_path_gz.txt", P, fmt="%.4f", header="x y z of every camera, Gazebo frame")
    np.savetxt(model / "T_slam_to_gazebo.txt", T_gz, fmt="%.8f")
    print(f"camera height above floor: median {np.median(P[:, 2]):.2f} m")
    print(f"\nwrote {model}/ and {worlds / (args.name + '.sdf')}")
    print(f"run:  GZ_SIM_RESOURCE_PATH={ROOT}/gazebo/models gz sim {worlds / (args.name + '.sdf')}")


if __name__ == "__main__":
    main()
