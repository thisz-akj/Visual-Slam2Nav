"""Render overview pictures of the Gazebo twin + robot for the guide (docs/figures/gz_*.png).

Adds two temporary static cameras to a copy of a robot world, runs Gazebo headless,
and saves one image from each.

    .venv/bin/python docs/render_gazebo_views.py
"""
import os
import signal
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "figures"

CAMS = """
    <model name="overview_cams">
      <static>true</static>
      <link name="top">
        <pose>{tx} {ty} {tz} 0 1.5708 0</pose>
        <sensor name="top_cam" type="camera">
          <topic>overview_top</topic><update_rate>2</update_rate><always_on>true</always_on>
          <camera><horizontal_fov>{tfov}</horizontal_fov><image><width>1400</width><height>1050</height></image>
            <clip><near>0.1</near><far>30</far></clip></camera>
        </sensor>
      </link>
      <link name="oblique">
        <pose>{ox} {oy} {oz} 0 {opitch} {oyaw}</pose>
        <sensor name="oblique_cam" type="camera">
          <topic>overview_oblique</topic><update_rate>2</update_rate><always_on>true</always_on>
          <camera><horizontal_fov>1.1</horizontal_fov><image><width>1400</width><height>1000</height></image>
            <clip><near>0.1</near><far>30</far></clip></camera>
        </sensor>
      </link>
    </model>
"""


def render(world, tag, top=(0.3, -0.2, 2.45, 2.3), oblique=(-1.0, -3.6, 2.35, 0.5, 1.25)):
    src = (ROOT / "gazebo/worlds" / f"{world}.sdf").read_text()
    cams = CAMS.format(tx=top[0], ty=top[1], tz=top[2], tfov=top[3], ox=oblique[0], oy=oblique[1],
                       oz=oblique[2], opitch=oblique[3], oyaw=oblique[4])
    tmp = ROOT / "gazebo/worlds" / f"_render_{world}.sdf"
    tmp.write_text(src.replace("  </world>", cams + "  </world>"))
    env = dict(os.environ, GZ_SIM_RESOURCE_PATH=str(ROOT / "gazebo/models"))
    for k in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR"):
        env.pop(k, None)
    sim = subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering", str(tmp)], env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    got = {}
    try:
        node = Node()
        for t in ("overview_top", "overview_oblique"):
            node.subscribe(Image, "/" + t, lambda m, t=t: got.__setitem__(
                t, np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy()))
        t0 = time.time()
        while len(got) < 2 and time.time() - t0 < 240:
            time.sleep(0.5)
        time.sleep(3)                                   # let the textures finish loading
        for t, im in got.items():
            cv2.imwrite(str(OUT / f"gz_{tag}_{t.split('_')[1]}.png"), cv2.cvtColor(im, cv2.COLOR_RGB2BGR))
            print("wrote", f"gz_{tag}_{t.split('_')[1]}.png")
    finally:
        os.killpg(sim.pid, signal.SIGINT)
        sim.wait(15)
        tmp.unlink()


if __name__ == "__main__":
    render("apt0_hd_robot", "hd", oblique=(2.1, -0.35, 1.45, 0.42, 3.1416))
