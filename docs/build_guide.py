"""Build the illustrated project guide PDF: docs/Visual_SLAM_Project_Guide.pdf

    .venv/bin/python docs/make_figures.py          # diagrams + result figures
    .venv/bin/python docs/render_gazebo_views.py   # Gazebo overview renders
    .venv/bin/python docs/build_guide.py
"""
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (Image, KeepTogether, PageBreak, Paragraph, Preformatted, SimpleDocTemplate,
                                Spacer, Table, TableStyle)
from reportlab.platypus.tableofcontents import TableOfContents
from reportlab.lib.utils import ImageReader

ROOT = Path(__file__).resolve().parents[1]
FIG = ROOT / "docs" / "figures"
OUT = ROOT / "docs" / "Visual_SLAM_Project_Guide.pdf"

FONTS = "/usr/share/fonts/truetype/noto/"
pdfmetrics.registerFont(TTFont("Sans", FONTS + "NotoSans-Regular.ttf"))
pdfmetrics.registerFont(TTFont("Sans-Bold", FONTS + "NotoSans-Bold.ttf"))
pdfmetrics.registerFont(TTFont("Sans-Italic", FONTS + "NotoSans-Italic.ttf"))
pdfmetrics.registerFont(TTFont("Sans-Semi", FONTS + "NotoSans-SemiBold.ttf"))
pdfmetrics.registerFont(TTFont("Mono", FONTS + "NotoSansMono-Regular.ttf"))
pdfmetrics.registerFontFamily("Sans", normal="Sans", bold="Sans-Bold", italic="Sans-Italic", boldItalic="Sans-Bold")

INK = colors.HexColor("#1f2328")
QUIET = colors.HexColor("#5b636e")
BLUE = colors.HexColor("#2f6fdf")
BLUE_T = colors.HexColor("#eaf1fd")
GREEN = colors.HexColor("#2e8b57")
GREEN_T = colors.HexColor("#e9f6ee")
ORANGE = colors.HexColor("#d9822b")
ORANGE_T = colors.HexColor("#fdf3e6")
RED_T = colors.HexColor("#fbeaea")
GREY_T = colors.HexColor("#f3f5f7")
RULE = colors.HexColor("#d5dae0")

W, H = A4
MARGIN = 18 * mm
TEXT_W = W - 2 * MARGIN

S = {
    "body": ParagraphStyle("body", fontName="Sans", fontSize=10.2, leading=15, textColor=INK, spaceAfter=6),
    "small": ParagraphStyle("small", fontName="Sans", fontSize=8.8, leading=12, textColor=QUIET),
    "cap": ParagraphStyle("cap", fontName="Sans-Italic", fontSize=8.8, leading=12, textColor=QUIET,
                          alignment=TA_CENTER, spaceBefore=3, spaceAfter=12),
    "h1": ParagraphStyle("h1", fontName="Sans-Bold", fontSize=20, leading=25, textColor=INK, spaceAfter=10),
    "h2": ParagraphStyle("h2", fontName="Sans-Bold", fontSize=13.5, leading=18, textColor=BLUE,
                         spaceBefore=12, spaceAfter=5, keepWithNext=1),
    "h3": ParagraphStyle("h3", fontName="Sans-Semi", fontSize=11, leading=15, textColor=INK, spaceBefore=8,
                         spaceAfter=3, keepWithNext=1),
    "bullet": ParagraphStyle("bullet", fontName="Sans", fontSize=10.2, leading=14.5, textColor=INK,
                             leftIndent=14, bulletIndent=3, spaceAfter=3),
    "box": ParagraphStyle("box", fontName="Sans", fontSize=10, leading=14.5, textColor=INK),
    "boxh": ParagraphStyle("boxh", fontName="Sans-Bold", fontSize=9.5, leading=13, textColor=BLUE, spaceAfter=2),
    "cell": ParagraphStyle("cell", fontName="Sans", fontSize=8.8, leading=11.8, textColor=INK),
    "cellh": ParagraphStyle("cellh", fontName="Sans-Bold", fontSize=8.8, leading=11.8, textColor=INK),
    "cellsm": ParagraphStyle("cellsm", fontName="Sans", fontSize=8.2, leading=10.6, textColor=INK),
    "cellhsm": ParagraphStyle("cellhsm", fontName="Sans-Bold", fontSize=8.2, leading=10.6, textColor=INK),
    "code": ParagraphStyle("code", fontName="Mono", fontSize=8.3, leading=11.2, textColor=INK),
    "toc1": ParagraphStyle("toc1", fontName="Sans", fontSize=11, leading=19, textColor=INK),
    "title": ParagraphStyle("title", fontName="Sans-Bold", fontSize=28, leading=34, textColor=INK),
    "subtitle": ParagraphStyle("subtitle", fontName="Sans", fontSize=13.5, leading=19, textColor=QUIET),
}


# ------------------------------------------------------------------ building blocks
def P(text, style="body"):
    return Paragraph(text, S[style])


def bullets(items):
    return [Paragraph(t, S["bullet"], bulletText="•") for t in items]


def numbered(items):
    return [Paragraph(t, S["bullet"], bulletText=f"{i}.") for i, t in enumerate(items, 1)]


def callout(text, title="In simple words", bg=BLUE_T, edge=BLUE):
    t = Table([[[P(title, "boxh"), P(text, "box")]]], colWidths=[TEXT_W])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), bg), ("LINEBEFORE", (0, 0), (0, -1), 3, edge),
                           ("LEFTPADDING", (0, 0), (-1, -1), 10), ("RIGHTPADDING", (0, 0), (-1, -1), 10),
                           ("TOPPADDING", (0, 0), (-1, -1), 7), ("BOTTOMPADDING", (0, 0), (-1, -1), 8)]))
    return KeepTogether([Spacer(1, 3), t, Spacer(1, 8)])


def lesson(text):
    s = ParagraphStyle("boxh_o", parent=S["boxh"], textColor=ORANGE)
    t = Table([[[Paragraph("Lesson learned", s), P(text, "box")]]], colWidths=[TEXT_W])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), ORANGE_T), ("LINEBEFORE", (0, 0), (0, -1), 3, ORANGE),
                           ("LEFTPADDING", (0, 0), (-1, -1), 10), ("RIGHTPADDING", (0, 0), (-1, -1), 10),
                           ("TOPPADDING", (0, 0), (-1, -1), 7), ("BOTTOMPADDING", (0, 0), (-1, -1), 8)]))
    return KeepTogether([Spacer(1, 3), t, Spacer(1, 8)])


PHOTOS = {"gz_hd_oblique.png", "gz_hd_top.png", "rgbd_frame.png", "orb_features.png", "apt0_frame.png",
          "fr3_map_topdown.png", "apt0_map_topdown.png", "sim_views.png", "leveling.png",
          "splg_matches.png", "orb_vs_splg_twin.png", "gz_splg_top.png", "cmp_map_error_apt0.png"}


def as_jpeg(path):
    """Photos and dense scatter plots compress far better as JPEG (diagrams stay PNG)."""
    import cv2
    out = FIG / "_jpg" / (path.stem + ".jpg")
    out.parent.mkdir(exist_ok=True)
    if not out.exists() or out.stat().st_mtime < path.stat().st_mtime:
        im = cv2.imread(str(path))
        if im.shape[1] > 2000:
            im = cv2.resize(im, (2000, int(im.shape[0] * 2000 / im.shape[1])), interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(out), im, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return out


def figure(name, caption, width=1.0, max_h=None):
    path = FIG / name
    if name in PHOTOS:
        path = as_jpeg(path)
    iw, ih = ImageReader(str(path)).getSize()
    w = TEXT_W * width
    h = w * ih / iw
    if max_h and h > max_h:
        h = max_h
        w = h * iw / ih
    return KeepTogether([Spacer(1, 4), Image(str(path), width=w, height=h), P(caption, "cap")])


def table(rows, widths, header=True, zebra=True, compact=False):
    cs, hs = ("cellsm", "cellhsm") if compact else ("cell", "cellh")
    data = [[P(c, hs if (header and i == 0) else cs) for c in r] for i, r in enumerate(rows)]
    t = Table(data, colWidths=[TEXT_W * w for w in widths], repeatRows=1 if header else 0)
    pad = 2 if compact else 3.5
    st = [("GRID", (0, 0), (-1, -1), 0.4, RULE), ("VALIGN", (0, 0), (-1, -1), "TOP"),
          ("LEFTPADDING", (0, 0), (-1, -1), 5), ("RIGHTPADDING", (0, 0), (-1, -1), 5),
          ("TOPPADDING", (0, 0), (-1, -1), pad), ("BOTTOMPADDING", (0, 0), (-1, -1), pad + 0.5)]
    if header:
        st.append(("BACKGROUND", (0, 0), (-1, 0), GREY_T))
    if zebra:
        for i in range(2 if header else 1, len(rows), 2):
            st.append(("BACKGROUND", (0, i), (-1, i), colors.HexColor("#fafbfc")))
    t.setStyle(TableStyle(st))
    if len(rows) <= 10:
        return [KeepTogether([t, Spacer(1, 8)])]
    return [t, Spacer(1, 8)]


def code(text):
    t = Table([[Preformatted(text.strip("\n"), S["code"])]], colWidths=[TEXT_W])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), GREY_T), ("LEFTPADDING", (0, 0), (-1, -1), 8),
                           ("TOPPADDING", (0, 0), (-1, -1), 6), ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
                           ("BOX", (0, 0), (-1, -1), 0.4, RULE)]))
    return [t, Spacer(1, 8)]


def formula(text):
    s = ParagraphStyle("f", parent=S["code"], fontSize=10, leading=14, alignment=TA_CENTER)
    return [Spacer(1, 2), Paragraph(text, s), Spacer(1, 8)]


def glue_headings(story):
    """Put each h2/h3 heading inside the KeepTogether block that follows it, so it never ends a page alone."""
    out = []
    for f in story:
        if (isinstance(f, KeepTogether) and out and isinstance(out[-1], Paragraph)
                and out[-1].style.name in ("h2", "h3")):
            out[-1] = KeepTogether([out[-1]] + list(f._content))
        else:
            out.append(f)
    return out


class Doc(SimpleDocTemplate):
    def beforeDocument(self):
        self.h1_count = 0                 # multiBuild lays out several passes: keys must repeat exactly

    def afterFlowable(self, f):
        if isinstance(f, Paragraph) and f.style.name == "h1":
            text = f.getPlainText()
            key = "h1-%d" % self.h1_count
            self.h1_count += 1
            self.canv.bookmarkPage(key)
            self.canv.addOutlineEntry(text, key, level=0)
            self.notify("TOCEntry", (0, text, self.page, key))


def on_page(c, doc):
    if doc.page == 1:
        return
    c.saveState()
    c.setFont("Sans", 8)
    c.setFillColor(QUIET)
    c.drawString(MARGIN, H - 11 * mm, "Visual SLAM to Autonomous Navigation")
    c.drawRightString(W - MARGIN, 10 * mm, str(doc.page))
    c.setStrokeColor(RULE)
    c.line(MARGIN, H - 12.5 * mm, W - MARGIN, H - 12.5 * mm)
    c.restoreState()


def H1(t):
    return Paragraph(t, S["h1"])


# ------------------------------------------------------------------ content
def build():
    st = []

    # ---------- cover
    st += [Spacer(1, 30 * mm), P("Visual SLAM to<br/>Autonomous Navigation", "title"), Spacer(1, 6),
           P("From real RGB-D video to a simulated apartment where a robot maps, is tested against "
             "perfect ground truth, and drives itself. Two SLAM pipelines, compared: ORB-SLAM3 and "
             "SuperPoint + LightGlue.", "subtitle"), Spacer(1, 10 * mm),
           figure("pipeline.png", "The whole project on one page: real video goes in at the top, a working "
                  "robot simulation comes out at the bottom."),
           Spacer(1, 6 * mm),
           P("Project guide and study notes &middot; Azad Kumar Jha &middot; 13 October 2026", "small"),
           P("Code: ~/github/visual_slam &middot; Tools: ORB-SLAM3, Open3D, Gazebo Harmonic, Python", "small"),
           PageBreak()]

    # ---------- contents
    toc = TableOfContents()
    toc.levelStyles = [S["toc1"]]
    st += [P("Contents", "h2"), toc, Spacer(1, 8 * mm),
           callout("Each chapter starts with an <b>In simple words</b> box like this one. Read only those boxes "
                   "for a 10-minute overview; read the rest when you want the details. Orange <b>Lesson learned</b> "
                   "boxes mark mistakes we made and how we caught them. A glossary of every technical word is at "
                   "the end.", title="How to read this guide"),
           PageBreak()]

    # ---------- 1 big picture
    st += [H1("1. The big picture"),
           callout("We took videos of real rooms, used them to build an accurate 3D copy of an apartment, put that "
                   "copy into a robot simulator, and then (a) tested how well a robot can map the room from its own "
                   "camera, and (b) made the robot drive itself to any point you click. We then built a second SLAM "
                   "pipeline with learned features (SuperPoint + LightGlue) and compared the two."),
           P("The project has seven stages. Each one produces something the next stage uses:"),
           *numbered(["<b>Record</b>: colour + depth video from public datasets (TUM and Stanford BundleFusion).",
                      "<b>Localize</b>: ORB-SLAM3 works out where the camera was for every frame.",
                      "<b>Map</b>: depth images are fused at those positions into a 3D mesh of the room.",
                      "<b>Twin</b>: the mesh is levelled, simplified and coloured, then turned into a Gazebo world "
                      "(a “digital twin” of the real apartment).",
                      "<b>Robot</b>: a small wheeled robot with a lidar, a camera and an IMU is placed inside.",
                      "<b>Test SLAM</b>: the robot drives a loop; ORB-SLAM3 maps from its camera and is scored "
                      "against the simulator's exact position.",
                      "<b>Navigate</b>: click a point on a floor plan; the robot plans a route and drives there, "
                      "avoiding surprise obstacles."]),
           Spacer(1, 4), P("Headline results", "h3"),
           *table([["What", "Result"],
                   ["ORB-SLAM3 on a real office video (22 m loop)", "1.0 cm average position error"],
                   ["ORB-SLAM3 on the real apartment scan (89 m)", "One continuous map; 9.7 cm vs reference"],
                   ["3D apartment model", "5.9 x 5.8 m, 1 cm detail, 1 million triangles, sharp colour texture"],
                   ["SLAM from the robot's camera in the simulator", "2.1 cm error over 15 m, never lost track"],
                   ["Pipeline 2 (SuperPoint + LightGlue)", "Never lost the map on any of 6 sequences (ORB-SLAM3 "
                                                            "broke on 3); about 30x slower on this CPU"],
                   ["Autonomous navigation", "Reaches clicked goals within about 10 cm; drives around a "
                                             "surprise box, or reports when the way is blocked"]],
                  [0.48, 0.52]),
           figure("gz_hd_oblique.png", "The final result: our robot (blue) inside the 3D copy of the real "
                  "apartment, rendered by Gazebo. The bookshelf, curtain, table and chairs all come from the scan.",
                  width=0.82),
           PageBreak()]

    # ---------- 2 basics
    st += [H1("2. The basics you need"),
           callout("A camera turns 3D points into 2D pixels. SLAM runs this backwards: from many photos it figures "
                   "out both <i>where the camera was</i> and <i>where things are</i>. It does this by spotting the "
                   "same small details (features) in many photos and finding the positions that explain them best."),
           P("2.1 How a camera sees", "h2"),
           P("A camera is like a pinhole: light from a 3D point travels in a straight line through the camera "
             "centre and lands on the image. The farther away the point (bigger Z), the closer to the centre of the "
             "image it lands. Four numbers describe a camera: two focal lengths (fx, fy, how zoomed-in it is) and "
             "the image centre (cx, cy). Together they are called the <b>intrinsics</b>."),
           figure("pinhole.png", "The pinhole camera model: a 3D point is projected through the camera centre "
                  "onto the image plane.", width=0.8),
           P("A normal camera loses depth: from one photo you cannot tell a small near object from a big far one. "
             "An <b>RGB-D camera</b> (like the Kinect or Structure Sensor) also measures depth for every pixel. "
             "That is why every main run in this project used RGB-D: the depth gives true sizes in metres."),
           figure("rgbd_frame.png", "One RGB-D frame from the TUM office dataset: the colour image and its depth "
                  "image. Each depth pixel is a distance in metres (stored as a 16-bit number; divide by 5000)."),
           *table([["Camera", "fx", "fy", "cx", "cy", "Depth unit"],
                   ["TUM fr3 (Kinect)", "535.4", "539.2", "320.1", "247.6", "1/5000 m"],
                   ["BundleFusion (Structure Sensor)", "582.9", "582.9", "320", "240", "1/1000 m"],
                   ["Gazebo robot camera (60° view)", "554.4", "554.4", "320", "240", "metres (float)"]],
                  [0.36, 0.11, 0.11, 0.11, 0.11, 0.2]),
           P("2.2 What SLAM does", "h2"),
           P("SLAM stands for <b>Simultaneous Localization And Mapping</b>. Localization needs a map, and building a "
             "map needs to know where you are, so SLAM does both at once in a loop:"),
           figure("slam_idea.png", "The SLAM loop: each new image is matched to the map, the pose is estimated, and "
                  "the map is improved, which makes the next pose easier.", width=0.9),
           P("2.3 Features: the details SLAM tracks", "h2"),
           P("SLAM does not use every pixel. It picks <b>features</b>: small, distinctive spots such as corners and "
             "edges of objects. ORB-SLAM3 uses <b>ORB features</b>. Each one gets a 256-bit “fingerprint” "
             "(descriptor), so the same spot can be recognised in another photo by comparing fingerprints."),
           figure("orb_features.png", "Top: ORB features found in a real frame. Bottom: the same features matched "
                  "in a frame taken half a second later. Blank walls give almost no features.", width=0.85),
           callout("Features need texture: patterns, edges, writing, objects. A smooth white wall or a blurry "
                   "surface has almost no features, and SLAM gets lost there. Remember this; it explains most of "
                   "the simulation results in chapter 8.", title="Why this matters later", bg=GREEN_T, edge=GREEN),
           P("2.4 Pose: position + rotation", "h2"),
           P("A camera's <b>pose</b> is where it is (x, y, z) and which way it is facing (3 rotation angles): "
             "6 numbers in total. It is stored as a 4x4 matrix. Rotations are tricky to optimize because you cannot "
             "simply add numbers to a rotation and still have a valid rotation. SLAM software therefore works with "
             "a small 6-number change (“perturbation”) and applies it with a special function "
             "(the exponential map, from Lie group theory). You do not need the maths to use SLAM, but it is the "
             "first lab of the from-scratch course in this project (labs/00_lie_groups.md)."),
           P("2.5 Finding the pose from known points (PnP)", "h2"),
           P("If the map already contains 3D points and we can see some of them in a new photo, we can compute "
             "where the camera must be. This is called <b>Perspective-n-Point (PnP)</b>. Some matches are always "
             "wrong, so <b>RANSAC</b> is used: try many small random sets of matches, keep the answer most matches "
             "agree with, and throw away the rest as outliers."),
           P("2.6 Bundle adjustment: polishing everything together", "h2"),
           P("Small errors build up. <b>Bundle adjustment (BA)</b> fixes this by adjusting all camera poses and all "
             "3D points together so that every point projects as close as possible to where it was actually seen. "
             "The “error” it minimizes is the distance in pixels between where a point should appear and "
             "where it was seen (the <b>reprojection error</b>)."),
           *formula("minimize:  sum of ( projected point - observed pixel )<super>2</super>"),
           P("2.7 Scale: why depth matters", "h2"),
           P("With a single normal camera, a scene and a copy of it twice as big look identical, so the map's "
             "<b>scale</b> is unknown and slowly drifts. In an early test the map shrank to 65% of its true size. "
             "An RGB-D camera measures real distance, so its maps have the correct size."),
           PageBreak()]

    # ---------- 3 ORB-SLAM3
    st += [H1("3. ORB-SLAM3: the SLAM engine"),
           callout("ORB-SLAM3 is a well-known, open-source SLAM system. It runs three jobs at the same time: one "
                   "follows the camera in every frame, one builds and polishes the map, and one recognises places it "
                   "has seen before to remove accumulated errors."),
           figure("orbslam_threads.png", "ORB-SLAM3's three threads and what each one does."),
           P("Key ideas", "h2"),
           *bullets(["<b>Keyframes</b>: only some frames are kept in the map (when the view has changed enough). "
                     "This keeps the map small and fast.",
                     "<b>Covisibility graph</b>: keyframes that see the same points are linked, so ORB-SLAM3 only "
                     "has to work with the nearby part of the map.",
                     "<b>Bag of words</b>: features are grouped into a “visual vocabulary” (about a million "
                     "words, file ORBvoc.txt). Each keyframe becomes a word list, so similar places can be found "
                     "instantly, like a search engine for images.",
                     "<b>Loop closure</b>: when the camera returns to a known place, the drift that built up along "
                     "the way is spread out and corrected over the whole path.",
                     "<b>Atlas and map merging</b>: if tracking is lost, ORB-SLAM3 starts a new map. If it later "
                     "recognises a place from an old map, it merges the two (“Merge detected”)."]),
           P("Getting it to build", "h2"),
           P("ORB-SLAM3 is from 2022 and did not compile on Ubuntu 24.04 out of the box. We installed the build "
             "tools, then fixed four errors: Pangolin (its 3D viewer) needed an extra header, both had to use the "
             "C++17 standard, one variable that was incremented had to change from bool to int, and we added our "
             "own function to save the map points (ORB-SLAM3 only saves trajectories). ORB-SLAM3 also tends to "
             "crash when its viewer closes, after it has saved everything, so our run script checks for the saved "
             "trajectory instead of trusting the exit code."),
           P("What it outputs", "h2"),
           *table([["File", "What is inside"],
                   ["CameraTrajectory.txt", "The pose of every frame: time, position, rotation (quaternion)"],
                   ["KeyFrameTrajectory.txt", "The poses of the keyframes only"],
                   ["MapPoints.txt", "The sparse 3D map (added by us): map id, x, y, z, number of observations"]],
                  [0.32, 0.68]),
           PageBreak()]

    # ---------- 4 first maps
    st += [H1("4. First maps from real video"),
           callout("We ran ORB-SLAM3 on two real recordings: an office (with a motion-capture system that knows "
                   "the true camera path) and a whole apartment. In the office, ORB-SLAM3's path was within 1 cm "
                   "of the truth on average over 22 metres."),
           P("4.1 How we measure accuracy", "h2"),
           P("The SLAM path and the true path are in different coordinate systems, so we first line them up "
             "(<b>Umeyama alignment</b>: the best rotation + shift, and for single-camera runs also a scale). Then "
             "<b>ATE</b> (Absolute Trajectory Error) is the average distance between matching points of the two "
             "paths (root-mean-square). Smaller is better."),
           P("4.2 The office: TUM fr3_long_office_household", "h2"),
           P("2,488 RGB-D frames, 2.5 minutes, the camera circles two desks and returns to the start. "
             "ORB-SLAM3 recognised the start and <b>closed the loop</b>."),
           figure("fr3_traj_vs_gt.png", "Black: the true camera path (motion capture). Red: ORB-SLAM3's estimate. "
                  "They overlap almost perfectly: 1.01 cm average error, 3 cm worst case, over 22.1 m.", width=0.62),
           figure("fr3_map_topdown.png", "Top view of the office map. Left: the dense coloured map with the camera "
                  "path. Right: ORB-SLAM3's sparse feature points."),
           P("4.3 The apartment: Stanford BundleFusion apt0", "h2"),
           P("8,560 frames from a Structure Sensor carried through a multi-room apartment, an 89 m walk. "
             "There is no motion capture here; the reference path comes from another system (BundleFusion), so the "
             "9.7 cm difference is agreement, not true error. ORB-SLAM3 kept one continuous map but did not detect "
             "a loop, so drift was not corrected."),
           figure("apt0_frame.png", "One frame of the apartment scan: colour and depth."),
           figure("apt0_map_topdown.png", "Top view of the apartment: rooms, sofa and furniture, with the "
                  "walking path in red."),
           *table([["Run", "Camera", "Error", "Notes"],
                   ["Our own early prototype, fr1_xyz", "Single", "8.2 cm", "Map shrank to 65% scale"],
                   ["ORB-SLAM3, office (fr3)", "RGB-D", "1.0 cm", "Loop closed; 34 ms per frame"],
                   ["ORB-SLAM3, apartment (apt0)", "RGB-D", "9.7 cm vs reference", "One map; 16 ms per frame"]],
                  [0.36, 0.13, 0.22, 0.29]),
           PageBreak()]

    # ---------- 5 dense mapping
    st += [H1("5. From points to a solid 3D model"),
           callout("ORB-SLAM3's map is just a cloud of feature points, good for navigation but not something you "
                   "can look at. To get solid walls and furniture, we take every depth image, place it where "
                   "ORB-SLAM3 says the camera was, and blend them all into one smooth surface."),
           P("The method is called <b>TSDF fusion</b> (Truncated Signed Distance Function). Space is divided into "
             "tiny cubes (<b>voxels</b>, 1 or 2 cm). Each voxel remembers how far it is from the nearest surface: "
             "positive on the camera's side, negative behind the surface. Every depth image updates these numbers "
             "with a running average, so sensor noise averages away. The surface is exactly where the numbers "
             "change from + to -. <b>Marching cubes</b> then turns that boundary into triangles."),
           figure("tsdf.png", "Left: a 2D slice of a TSDF. The black line is the real surface; voxel numbers are "
                  "distances to it. Right: the fusion steps."),
           *table([["Map", "Voxel size", "Frames used", "Triangles", "Time"],
                   ["Office (fr3)", "1 cm", "830", "1.1 million", "18 s"],
                   ["Apartment (apt0)", "2 cm", "2,854", "0.9 million", "about 2 min"],
                   ["Apartment HD", "1 cm", "2,854", "4.2 million", "60 s"]],
                  [0.26, 0.16, 0.18, 0.2, 0.2]),
           P("Smaller voxels give more detail but four times more triangles each time the size halves. "
             "Chapter 8 shows that this detail turned out to matter a lot."),
           PageBreak()]

    # ---------- 6 twin
    st += [H1("6. Building the digital twin"),
           callout("The raw 3D model is tilted (it uses the first camera's direction as “up”), too heavy "
                   "for a simulator, and coloured in a way Gazebo cannot show. Six steps turn it into a clean, "
                   "level, light Gazebo model of the apartment."),
           figure("twin_steps.png", "The six export steps in tools/export_gazebo.py."),
           *bullets(["<b>Clean</b>: delete small floating pieces (948 of 953 pieces were noise).",
                     "<b>Level</b>: find the floor with <b>RANSAC plane fitting</b> and rotate it flat. “Down” "
                     "is estimated as the camera's down direction averaged over the whole walk (people hold a "
                     "scanner roughly upright on average). The floor is the lowest large horizontal plane, so a "
                     "big table is not mistaken for it.",
                     "<b>Square</b>: rotate so the walls line up with the x and y axes, and centre the apartment.",
                     "<b>Simplify</b>: reduce triangles (quadric decimation) while keeping the shape. One mesh for "
                     "looks (1 million triangles) and a lighter one for physics (0.1 million).",
                     "<b>Texture</b>: Gazebo shows colour from an image, not from points, so colours are “baked” "
                     "into an 8192 x 8192 image.",
                     "<b>Package</b>: write the Gazebo model files and a world file."]),
           figure("leveling.png", "Side view of the apartment before and after levelling. After: floor at 0 m, "
                  "ceiling at 2.75 m, and the scanner path 1.45 m above the floor (chest height), a good sanity "
                  "check."),
           lesson("The first levelling attempt put the whole apartment 1.4 m <i>underground</i> because of a sign "
                  "error in one formula. It was caught because the camera height came out negative (-1.07 m). "
                  "Always compare outputs with physical common sense: room height, person height, floor at zero."),
           P("Texture without the headache", "h3"),
           P("The standard texturing tool (UVAtlas) needs a perfectly clean mesh and took more than 10 minutes on "
             "1 million triangles. We wrote a faster method: give every triangle its own small square in the image "
             "and colour it from the full-detail 4-million-triangle model. Triangles that are neighbours in the "
             "room are placed next to each other in the image (a <b>Morton / Z-order</b> curve), so blending at "
             "the edges mixes similar colours and no seams appear."),
           figure("gz_hd_top.png", "The finished twin in Gazebo, seen from just under the ceiling. The robot is the "
                  "small blue square in the middle.", width=0.82),
           PageBreak()]

    # ---------- 7 Gazebo
    st += [H1("7. Gazebo: the robot simulator"),
           callout("Gazebo is a free robot simulator. It handles physics (wheels, collisions), draws what cameras "
                   "and lidars would see, and lets programs talk to the robot through named message channels "
                   "called <b>topics</b>. We used Gazebo Harmonic, the current long-term release."),
           figure("gazebo_arch.png", "What runs inside Gazebo, and the topics our Python tools use to read sensors "
                  "and send drive commands."),
           P("7.1 Worlds and models (SDF files)", "h2"),
           P("Everything in Gazebo is described in <b>SDF</b>, a simple XML format. A <b>model</b> is a folder with "
             "model.sdf (shapes, joints, sensors) and model.config (name). A <b>world</b> file lists the physics "
             "settings, the light, the GUI layout and which models to include. The apartment is a <i>static</i> "
             "model: it never moves. It has a visual mesh (what you see) and a collision mesh (what the robot "
             "bumps into)."),
           P("7.2 The robot: scout_bot", "h2"),
           figure("robot.png", "Left: the robot from above. Right: the two camera heights we tested.", width=0.9),
           *table([["Part", "Gazebo element", "Topic", "Details"],
                   ["Wheels", "DiffDrive plugin", "/cmd_vel (in), /odom (out)", "2 driven wheels, r = 5 cm"],
                   ["Lidar", "gpu_lidar sensor", "/scan", "360 beams, 10 per second, up to 8 m"],
                   ["Camera", "rgbd_camera sensor", "/camera/image, /camera/depth_image", "640 x 480, 15 per second"],
                   ["IMU", "imu sensor", "/imu", "100 per second"],
                   ["True position", "PosePublisher plugin", "/model/scout_bot/pose", "Exact pose, 100 per second"]],
                  [0.14, 0.22, 0.36, 0.28]),
           P("7.3 Talking to Gazebo from Python", "h2"),
           P("Gazebo's message system (<b>gz-transport</b>) has Python bindings. Our scripts subscribe to topics "
             "(camera images, lidar scans, the true pose) and publish to /cmd_vel to drive. Services are used to "
             "add and remove objects while running (we dropped test boxes this way) and to draw the planned route "
             "in the 3D view (the /marker service, provided by the MarkerManager GUI plugin)."),
           *code("""
gz topic -l                                   # list all topics
gz topic -e -t /scan -n 1                     # print one lidar message
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.2}'   # drive forward
gz sim -s -r --headless-rendering world.sdf   # run without a window (used for all tests)
"""),
           P("7.4 GUI panels used", "h2"),
           P("3D view, play/pause, world stats, the robot camera image (ImageDisplay), lidar rays "
             "(VisualizeLidar), keyboard driving (KeyPublisher with the arrow keys), and route drawing "
             "(MarkerManager). Right-click the robot and choose <b>Follow</b> to keep it in view."),
           PageBreak()]

    # ---------- 8 SLAM in the twin
    st += [H1("8. Testing SLAM inside the twin"),
           callout("In real life you never know exactly where a camera was. In the simulator you do. So we drove "
                   "the robot around the apartment, fed its camera images to ORB-SLAM3, and compared ORB-SLAM3's "
                   "answer with the exact truth. Then we changed one thing at a time to learn what makes SLAM "
                   "succeed or fail."),
           figure("sim_experiment.png", "One test run, start to finish (tools/sim_collect.py, run_orbslam3.sh, "
                  "eval_segments.py)."),
           figure("sim_plan.png", "The planned test loop: 8 spread-out waypoints, visited in order and back to "
                  "the start, on the floor plan made from the scan.", width=0.5),
           P("8.1 The four experiments", "h2"),
           *table([["Run", "What changed", "Lost track", "Longest stretch tracked", "Error while tracking"],
                   ["A", "Starting point: camera at 0.37 m, 2 cm model", "5 times", "3.4 m (20%)", "about 1 cm"],
                   ["B", "Slower turns", "5 times, same places", "3.5 m (20%)", "about 1 cm"],
                   ["D", "Camera raised to 1.2 m", "3 times (2 maps merged)", "5.5 m (36%)", "about 1 cm"],
                   ["C", "1.2 m camera + 1 cm HD model", "never", "14.9 m (100%)", "2.1 cm over the whole loop"]],
                  [0.07, 0.33, 0.18, 0.2, 0.22]),
           figure("sim_results.png", "Left to right: how often tracking was lost, how much of the loop was tracked "
                  "without a break, and how many features the hardest 10% of frames had (orange: the real "
                  "footage)."),
           figure("sim_views.png", "What the robot's camera saw in each run. The low camera (A) sees holes and "
                  "undersides the human scanner never filmed. The HD model (C) is sharp enough to read the poster."),
           P("8.2 What each run taught us", "h2"),
           *bullets(["<b>A</b>: when ORB-SLAM3 was tracking, it was accurate (about 1 cm). The problem was that it "
                     "kept losing track.",
                     "<b>B</b>: slowing down did nothing; it lost track at the very same spots. So the cause was the "
                     "<i>scene</i>, not the motion.",
                     "<b>D</b>: from 37 cm high, the camera saw surfaces the scanner (carried at 1.45 m) never filmed: "
                     "holes, ragged edges, grey gaps. At 1.2 m it saw what the scanner saw, features doubled, and "
                     "ORB-SLAM3 could even recognise places again.",
                     "<b>C</b>: the last problem was blurry texture. With the sharper 1 cm model, tracking never "
                     "broke."]),
           callout("A scanned copy of a room is only trustworthy for testing a robot if it was scanned from the "
                   "robot's own viewpoint and in enough detail. Fix both, and simulated SLAM behaves like real SLAM.",
                   title="The main finding", bg=GREEN_T, edge=GREEN),
           lesson("Our first attempt at run C looked perfect (3.1 cm, 1,500 features in every frame). It was wrong: "
                  "a texture bug made every triangle show another triangle's colours, like confetti, and the fake "
                  "detail fooled ORB-SLAM3. We caught it because the simulation had <i>more</i> detail than the real "
                  "video, which is impossible for a copy. When a result looks too good, check it against reality."),
           P("One honest caveat: the simulator is now a little <i>too</i> easy. Its camera has no motion blur, "
             "noise or exposure changes, so its worst frames still have more features than real ones (1,037 vs "
             "583)."),
           PageBreak()]

    # ---------- 9 navigation
    st += [H1("9. Making the robot drive itself"),
           callout("You click a point on a floor plan. The robot works out the shortest safe route (A*), follows it "
                   "smoothly (pure pursuit), and keeps an eye on its lidar. If something unexpected is in the way, "
                   "it stops, adds it to its map and finds another route, or tells you the goal cannot be reached."),
           figure("nav_loop.png", "The navigation loop in tools/navigate.py."),
           P("9.1 A floor plan from the 3D scan", "h2"),
           P("The 3D model is sliced into 5 cm squares. Floor squares are free; anything between 4 cm and 50 cm "
             "high (the robot's height) is an obstacle; floor that was never scanned is treated as unsafe. Then "
             "each free square gets its <b>clearance</b>: the distance to the nearest obstacle. The robot is about "
             "40 cm wide, so only squares with at least 20 cm clearance are drivable. This trick (called "
             "<b>inflation</b>) lets the planner treat the robot as a single point."),
           figure("nav_grid.png", "Left: the floor plan the robot uses. Right: clearance, brighter = more room "
                  "around you."),
           P("9.2 A*: finding the route", "h2"),
           P("A* searches the grid from the robot to the goal, always expanding the most promising square first "
             "(distance travelled so far + straight-line distance still to go). This finds the shortest path "
             "quickly. We also make squares near obstacles more “expensive”, so routes stay in the middle "
             "of corridors instead of scraping past furniture."),
           figure("nav_route.png", "A route planned between two rooms of the apartment.", width=0.55),
           P("9.3 Pure pursuit: following the route", "h2"),
           P("Twenty times a second, the robot picks a point on the route 35 cm ahead and steers along the circle "
             "arc that reaches it, like a driver looking a little down the road instead of at their feet. For sharp "
             "corners it turns on the spot first, and it slows down near the goal."),
           figure("pure_pursuit.png", "Pure pursuit: steer along the arc that reaches the look-ahead point.",
                  width=0.6),
           P("9.4 Surprise obstacles", "h2"),
           P("If the lidar sees anything closer than 30 cm straight ahead that is not on the map, the robot stops, "
             "backs off a little, marks it on its map and plans again to the <i>same</i> goal. If the obstacle cuts "
             "off the goal completely, it stops and says BLOCKED."),
           *table([["Test", "Result"],
                   ["Drive to goals in different rooms (5.7 m and 6.8 m routes)", "Arrived within about 10 cm"],
                   ["A box dropped onto the only corridor to the goal", "Stopped and reported BLOCKED"],
                   ["A box dropped onto the route in an open area", "Replanned 4 times as the lidar saw more of the "
                                                                    "box; arrived at the real goal"]],
                  [0.55, 0.45]),
           lesson("An early version quietly swapped an unreachable goal for the nearest reachable spot and then "
                  "reported “Arrived”. A robot must never change its destination without telling you."),
           P("9.5 Using it", "h2"),
           P("Run <font face='Mono'>.venv/bin/python tools/navigate.py</font>. Gazebo and a map window open. Click "
             "on the map to send the robot; the route also appears as a green line in Gazebo. Keys: S or Space = "
             "stop, C = forget lidar obstacles, Q = quit. Note: the robot currently knows its exact position from "
             "the simulator; using ORB-SLAM3 for this is the next step."),
           PageBreak()]

    # ---------- 10 pipeline 2
    st += [H1("10. A second SLAM: SuperPoint + LightGlue"),
           callout("ORB features are hand-designed rules for finding corners. SuperPoint and LightGlue are neural "
                   "networks that <i>learned</i> to find and match image points from huge numbers of examples. We "
                   "built a second complete pipeline around them (pipeline 2) to see whether learned features fix the "
                   "places where ORB-SLAM3 lost track."),
           P("10.1 SuperPoint: learned keypoints", "h2"),
           P("SuperPoint is a small convolutional neural network (CNN). One pass over the image produces two "
             "things: a heat map of where good keypoints are, and a 256-number descriptor for every keypoint. It "
             "was trained without manual labels: it learned which points stay recognisable when the same image is "
             "warped, rotated and re-lit in many random ways. That is why it also finds usable points on softer "
             "edges and gradients, not only on sharp corners like ORB."),
           P("10.2 LightGlue: learned matching", "h2"),
           P("ORB matching compares each descriptor on its own and keeps a match only if it clearly beats the "
             "second-best (ratio test). LightGlue is a <b>transformer</b>: it looks at all keypoints of both images "
             "together and lets them exchange information (<b>attention</b>), so a point is matched using the "
             "context of its neighbours, like recognising a word from the sentence around it. It stops early when "
             "the answer is already clear, which saves time on easy image pairs."),
           figure("splg_matches.png", "SuperPoint keypoints and LightGlue matches on the same real frame pair as the "
                  "ORB figure in chapter 2.", width=0.85),
           P("10.3 How pipeline 2 works", "h2"),
           P("ORB-SLAM3 is hard-wired to ORB (even its place-recognition vocabulary is built from ORB descriptors), "
             "so SuperPoint cannot simply be plugged in. We wrote a new RGB-D SLAM in Python (folder "
             "<font face='Mono'>splg_slam/</font>) with the same overall design, so the comparison is about the "
             "features:"),
           figure("splg_arch.png", "Pipeline 2: the per-frame tracking path (top) and the keyframe, loop and "
                  "optimization steps (bottom)."),
           *bullets(["<b>Tracking</b>: match the new frame to the current keyframe with LightGlue. The keyframe's "
                     "matched points get 3D positions from its depth image, so PnP + RANSAC gives the new camera "
                     "pose directly. If that fails, try the previous frame.",
                     "<b>Keyframes</b>: a new keyframe when the overlap drops or the camera has moved more than 30 cm "
                     "or turned more than 20\u00b0. Each new keyframe is also linked to up to two nearby older ones.",
                     "<b>Loop closure</b>: candidate old keyframes are picked by overall image similarity and by "
                     "distance, then checked with LightGlue + PnP (at least 45 agreeing matches).",
                     "<b>Pose graph</b>: every link (\u201cedge\u201d) says how two keyframes are placed relative to "
                     "each other. When a loop is found, all keyframe poses are adjusted together so they agree with "
                     "every edge as well as possible. Frames are stored relative to their keyframe, so the whole "
                     "path moves with the correction.",
                     "<b>Lost and new maps</b>: if tracking fails, try to relocalize against old keyframes; after 3 "
                     "failed frames start a new map, and merge it into the old one if a loop links them later "
                     "(the same idea as ORB-SLAM3's Atlas)."]),
           P("10.4 Making it fast enough on a laptop CPU", "h2"),
           P("These networks are built for graphics cards; this laptop has none. Two measured tricks made pipeline 2 "
             "usable at about 0.3 to 0.5 s per frame:"),
           *table([["Setting", "SuperPoint time", "Why"],
                   ["Full image (640 px), 14 threads", "about 1.6 s", "The CPU has 4 fast and 10 slow cores; using all slows it down"],
                   ["Full image (640 px), 8 threads", "about 0.9 s", "Sweet spot for this CPU"],
                   ["Image resized to 480 px, 8 threads", "about 0.2 s", "Much faster convolution kernels at this size; keypoints "
                                                                           "are mapped back to full-resolution pixels"]],
                  [0.34, 0.2, 0.46]),
           P("10.5 From pipeline 2 to navigation", "h2"),
           P("Pipeline 2 writes exactly the same files as ORB-SLAM3 (camera trajectory, keyframes, map points), so "
             "every later step works unchanged: dense map, digital twin, Gazebo world and navigation. The apartment "
             "map from pipeline 2 became its own Gazebo twin (<font face='Mono'>apt0_splg</font>). Its automatic "
             "levelling and centring landed within 2 cm of the ORB-SLAM3 twin's, and the robot drove itself to two "
             "test goals in it, arriving 11 to 12 cm from each."),
           figure("gz_splg_top.png", "The apartment twin built from pipeline 2's poses, rendered in Gazebo (2 cm "
                  "version). The robot is the small blue square.", width=0.75),
           PageBreak()]

    # ---------- 11 comparison
    st += [H1("11. Comparing the two pipelines"),
           callout("Both pipelines ran on exactly the same frames of six sequences. Pipeline 2 never lost the map; "
                   "ORB-SLAM3 broke into 5 to 7 pieces on three of the four twin runs. On easy, well-textured real "
                   "video ORB-SLAM3 was more accurate and about 30 times faster. The final 3D maps were equally good, "
                   "and both drove the robot."),
           P("11.1 A fair test", "h2"),
           *bullets(["Same frames for both: every 2nd frame of the 30 Hz real videos, every frame of the 15 Hz twin "
                     "recordings (folder <font face='Mono'>runs/cmp/assoc/</font>).",
                     "Same scoring: the per-segment ATE tool from chapter 8; twin runs against exact Gazebo truth, the "
                     "office against motion capture, the apartment against BundleFusion's poses.",
                     "One command runs everything: <font face='Mono'>tools/compare_all.sh</font>, then "
                     "<font face='Mono'>tools/compare_report.py</font> makes the tables and charts."]),
           figure("cmp_summary.png", "Left: how much of each path was tracked without a break. Right: position "
                  "error (hatched = ORB-SLAM3 broke into several maps; the bar is the average over the pieces)."),
           P("11.2 Results", "h2"),
           *table([["Dataset", "ORB-SLAM3", "SuperPoint + LightGlue"],
                   ["Real office (motion capture)", "<b>1.56 cm</b>, 1 loop, 18 ms/frame", "2.66 cm, 34 loops, 296 ms/frame"],
                   ["Twin A: camera 0.37 m", "5 breaks, longest 20%", "<b>0 breaks, 100%, 2.07 cm</b>"],
                   ["Twin B: + slow turns", "6 breaks, longest 20%", "<b>0 breaks, 100%, 1.30 cm</b>"],
                   ["Twin D: camera 1.2 m", "4 breaks (+2 merges), longest 36%", "<b>0 breaks, 100%, 1.33 cm</b>"],
                   ["Twin C: camera 1.2 m + HD twin", "0 breaks, 2.75 cm", "0 breaks, <b>1.19 cm</b>"],
                   ["Real apartment (reference poses)", "10.75 cm, worst 76.8 cm, 5 loops", "<b>8.29 cm, worst 17.2 cm</b>, 643 loops"]],
                  [0.3, 0.34, 0.36]),
           P("11.3 Why learned features held on", "h2"),
           figure("orb_vs_splg_twin.png", "The same two blurry twin frames from run A, just before one of ORB-SLAM3's "
                  "tracking breaks. ORB finds 51 matches, almost all along two edges; LightGlue finds 191, spread over "
                  "the soft curtain folds.", width=0.85),
           *bullets(["<b>Blurry, low-detail texture</b>: ORB needs sharp corners. SuperPoint still finds repeatable "
                     "points on soft gradients, and LightGlue matches them using their context.",
                     "<b>More, better-spread matches</b> give PnP a stable pose, so tracking never broke.",
                     "<b>Much more loop closure</b>: 643 verified loops on the apartment against 5, which kept the worst "
                     "error at 17 cm instead of 77 cm."]),
           figure("cmp_traj_sim_A.png", "Twin run A. Left: ORB-SLAM3's path breaks into six pieces (red crosses; each "
                  "piece aligned separately, which flatters it). Right: pipeline 2 keeps one map around the whole loop."),
           figure("cmp_traj_fr3_office.png", "Real office video: both pipelines follow the motion-capture path; "
                  "ORB-SLAM3 is a little more precise here (1.6 vs 2.7 cm)."),
           P("11.4 Map quality", "h2"),
           P("Each pipeline's poses were fused into a 2 cm apartment map and compared with a map fused from the "
             "reference poses."),
           *table([["Pipeline", "Mean surface error", "Map within 5 cm of reference", "Reference covered"],
                   ["ORB-SLAM3", "5.55 cm", "59%", "63%"],
                   ["SuperPoint + LightGlue", "5.63 cm", "57%", "59%"]],
                  [0.3, 0.22, 0.26, 0.22]),
           figure("cmp_map_error_apt0.png", "Top view of both apartment maps, coloured by distance to the reference "
                  "surface (green = close, red = far). They are practically identical.", width=0.92),
           lesson("A better trajectory did not give a better map. ORB-SLAM3's 77 cm error spike lasted only a few "
                  "frames, and fusing thousands of depth images averaged it away. Always measure the thing you "
                  "actually use (here, the map), not only the number that is easiest to compute."),
           P("11.5 Honest caveats", "h2"),
           *bullets(["Speed is not like-for-like: ORB-SLAM3 runs mapping and loop closing in parallel C++ threads; "
                     "pipeline 2 runs everything in one Python thread, on a CPU instead of the GPU the networks were "
                     "designed for.",
                     "Pipeline 2's back end is simpler: it optimizes only keyframe poses, not the 3D points (no full "
                     "bundle adjustment). That is the likely reason ORB-SLAM3 is more precise on the easy office video.",
                     "Pipeline 2 is new code, tuned on just 200 office frames; ORB-SLAM3 is a mature, published system.",
                     "SuperPoint's pretrained weights are licensed for non-commercial use only."]),
           PageBreak()]

    # ---------- 10 problems
    st += [H1("12. Problems we hit and how we fixed them"),
           callout("Twenty-four problems came up. Most were ordinary (old code, library bugs). The most valuable ones "
                   "were caught only because a number did not make physical sense."),
           *table([["#", "Problem (what we saw)", "Cause", "Fix"],
                   ["1", "Nothing would build", "No compiler or CMake installed; needed sudo", "Installed build tools from a normal terminal"],
                   ["2", "Viewer (Pangolin) failed to compile", "Newer compiler needs an explicit header", "Added the missing header with a compiler flag"],
                   ["3", "ORB-SLAM3 failed against the viewer", "Different C++ versions", "Compiled both as C++17"],
                   ["4", "Error: ++ on a true/false variable", "Not allowed in C++17", "Changed the variable to a number"],
                   ["5", "Our map-export function did not compile", "Placed outside ORB-SLAM3's namespace", "Moved it inside"],
                   ["6", "Dense map came out empty", "Bug in an old Open3D class; minimum-weight setting", "Switched to Open3D's newer VoxelBlockGrid"],
                   ["7", "Plot crashed on colours", "Averaged colours slightly above 1.0", "Clamped colours to 0 to 1"],
                   ["8", "Apartment converter crashed", "A pattern did not allow a dot in a name", "Fixed the pattern"],
                   ["9", "Map step skipped after SLAM", "ORB-SLAM3 crashes when its viewer closes", "Continue if the trajectory file exists"],
                   ["10", "“No floor found”", "First frame tilted 45°, wrong idea of down", "Average down over the whole walk"],
                   ["11", "Apartment 1.4 m underground", "Sign error in the levelling formula", "Fixed; checked floor, ceiling, camera height"],
                   ["12", "Robot skipped half its loop", "Follower jumped ahead where the route doubles back", "Look at most 0.8 m ahead on the route"],
                   ["13", "Robot stuck on open floor", "Bump in the scanned floor", "Flat physics floor; scan kept for looks"],
                   ["14", "Planner found no floor", "It used the mesh whose floor we removed", "Plan from the visual mesh"],
                   ["15", "Big model loads slowly", "160 MB file", "Wait up to 3 minutes for Gazebo"],
                   ["16", "Texturing failed / too slow", "Unclean mesh; slow tool at 1M triangles", "Wrote a fast per-triangle texture baker"],
                   ["17", "Too-good result (confetti texture)", "Texture image read upside down by Gazebo", "Flipped it; marked the run INVALID and redid it"],
                   ["18", "Robot “arrived” at the wrong place", "Planner silently changed the goal", "Keep the real goal; report BLOCKED"],
                   ["19", "Gazebo crashed when navigation started", "OpenCV's Qt settings leaked into Gazebo", "Removed those settings for Gazebo"],
                   ["20", "Our own shell commands died", "A kill command matched itself", "Used exact process names"],
                   ["21", "SuperPoint took 1.6 s per image", "Hybrid CPU: slow cores dragged the fast ones down", "8 threads + 480 px images: 0.2 s"],
                   ["22", "Pipeline 2 slowed to 2.3 s per frame", "Pose-graph cost computed edge by edge in Python", "Vectorized with NumPy; checked on a synthetic graph"],
                   ["23", "One ORB-SLAM3 comparison run missing", "Camera-config path prefixed twice", "Resolved paths properly; re-ran after the batch"],
                   ["24", "Robot world needed manual edits for each new twin", "Spawn point depends on the twin", "New tool: make_robot_world.py picks an open spot"]],
                  [0.05, 0.31, 0.32, 0.32]),
           P("Three lessons to keep", "h2"),
           *bullets(["<b>Sanity-check every number against the real world.</b> Negative camera height, a room 2.75 m "
                     "tall, a copy with more detail than the original: these checks caught bugs that every program "
                     "step had silently accepted.",
                     "<b>Change one thing per experiment.</b> Only because run B changed nothing but speed could we "
                     "rule speed out.",
                     "<b>Never let a planner change the goal without saying so.</b>"]),
           PageBreak()]

    # ---------- 11 run
    st += [H1("13. How to run everything"),
           P("All commands run from <font face='Mono'>~/github/visual_Slam2Nav</font>."),
           *code("""
# 1. Build ORB-SLAM3 (once)
tools/build_orbslam3.sh

# 2. SLAM + dense map on the office video (live viewer)
tools/run_orbslam3.sh

# 3. The apartment: convert the dataset, then SLAM + 2 cm map
.venv/bin/python tools/prepare_bundlefusion.py data/apt0/apt0
MAP_ARGS="--voxel 0.02 --depth-max 4" tools/run_orbslam3.sh \\
    data/apt0/apt0 data/apt0/apt0/associations.txt data/apt0/apt0.yaml runs/apt0

# 4. Look at a map in 3D
.venv/bin/python tools/view_map.py --run runs/apt0

# 5. Make the HD digital twin for Gazebo
.venv/bin/python tools/export_gazebo.py --run runs/apt0_1cm --name apt0_hd \\
    --align-like apt0 --visual-tris 1000000 --tex-size 8192 --texture per-triangle

# 6. Drive the robot yourself (arrow keys, S = stop)
tools/run_gazebo.sh apt0_hd_robot

# 7. Record a SLAM test drive, run SLAM on it, score it
.venv/bin/python tools/sim_collect.py --out data/sim_run --gui --map apt0_hd \\
    --world apt0_hd_robot_tall --cam-z 1.20 --obst-top 1.3
tools/run_orbslam3.sh data/sim_run data/sim_run/associations.txt \\
    data/sim_run/camera.yaml runs/sim_run
.venv/bin/python tools/eval_segments.py --run runs/sim_run --gt data/sim_run/groundtruth.txt

# 8. Click-to-go navigation
.venv/bin/python tools/navigate.py
"""),
           P("Pipeline 2 (SuperPoint + LightGlue) on the apartment", "h3"),
           *code("""
# SLAM + dense map (about 1 hour on this CPU; same arguments as run_orbslam3.sh, last one = frame stride)
MAP_ARGS="--voxel 0.02 --depth-max 4 --every 2" tools/run_splg.sh \\
    data/apt0/apt0 data/apt0/apt0/associations.txt data/apt0/apt0.yaml runs/splg_apt0 2

# Gazebo twin + robot world, then click-to-go navigation
.venv/bin/python tools/export_gazebo.py --run runs/splg_apt0 --name apt0_splg
.venv/bin/python tools/make_robot_world.py --map apt0_splg
.venv/bin/python tools/navigate.py --world apt0_splg_robot --map apt0_splg

# Re-run the whole comparison (both pipelines, 6 sequences) and its report
tools/compare_all.sh && .venv/bin/python tools/compare_report.py
"""),
           *table([["Folder / file", "What it holds"],
                   ["tools/", "All pipeline scripts (build, run, map, export, collect, evaluate, navigate)"],
                   ["splg_slam/", "Pipeline 2: SuperPoint + LightGlue front end, tracking, loops, pose graph"],
                   ["gazebo/models/", "apt0, apt0_hd, apt0_splg (apartment twins), scout_bot, scout_bot_tall (robots)"],
                   ["gazebo/worlds/", "Worlds with and without the robot, normal and HD"],
                   ["data/", "Downloaded datasets and simulator recordings"],
                   ["runs/", "ORB-SLAM3 outputs, dense maps, evaluations"],
                   ["results/REPORT.md, COMPARISON.md", "Every result table; pipeline 1 vs pipeline 2"],
                   ["docs/", "This guide, its figures and the scripts that make them"],
                   ["ROADMAP.md, labs/, vslam/", "The from-scratch learning course (Lab 0 is ready)"]],
                  [0.3, 0.7]),
           PageBreak()]

    # ---------- 12 next
    st += [H1("14. What is next"),
           P("The project works end to end. These are the most valuable next steps, in order:"),
           *numbered(["<b>Save the work in git</b> as its own repository.",
                      "<b>Navigate using ORB-SLAM3</b> instead of the simulator's perfect position. This closes the "
                      "loop from SLAM to navigation, the way a real robot works.",
                      "<b>Make the simulated camera realistic</b>: add motion blur, noise and exposure changes, so "
                      "the simulator is not easier than reality.",
                      "<b>Test loop closure</b> with routes that revisit places.",
                      "<b>Move to ROS 2 + Nav2</b>, the standard robot navigation stack, using the same twin.",
                      "<b>Add local bundle adjustment to pipeline 2</b> (refine 3D points too), the likely fix for "
                      "its lower precision on easy video.",
                      "<b>Run pipeline 2 on a GPU</b>, where SuperPoint and LightGlue are designed to run.",
                      "<b>Finish the from-scratch labs</b> and compare your own SLAM against both pipelines in the twin."]),
           Spacer(1, 6),
           P("Known limits today", "h2"),
           *bullets(["Navigation and test recording use the simulator's exact position.",
                     "The apartment twin is frozen: things that moved during the scan are fixed in place, and some "
                     "floating scan fragments remain.",
                     "The apartment's reference path is another system's estimate, not true ground truth.",
                     "ORB-SLAM3 closed few loops on the apartment and none in the simulator loops; pipeline 2 closes "
                     "many but is far slower on this CPU."]),
           PageBreak()]

    # ---------- glossary
    gl = [["Word", "Meaning"],
          ["A* (A-star)", "A search method that finds the shortest path on a grid quickly"],
          ["ATE", "Absolute Trajectory Error: average distance between the estimated and true paths"],
          ["Bag of words", "Describing an image by which “visual words” (feature types) it contains, for fast place search"],
          ["Attention / transformer", "A neural-network layer that lets every item look at all the others; LightGlue is built from these"],
          ["Bundle adjustment (BA)", "Adjusting all camera poses and 3D points together to reduce reprojection error"],
          ["CNN", "Convolutional neural network: a network that slides small learned filters over an image"],
          ["Clearance / inflation", "Distance to the nearest obstacle; growing obstacles by the robot's radius"],
          ["Collision mesh", "Simplified shape used for physics (what the robot bumps into)"],
          ["Covisibility graph", "Links between keyframes that see the same map points"],
          ["Depth image", "An image where each pixel is a distance instead of a colour"],
          ["Descriptor", "A short binary “fingerprint” of a feature, used for matching"],
          ["Digital twin", "A simulated copy of a real place or object"],
          ["Drift", "Error that slowly builds up over time in SLAM"],
          ["Feature (ORB)", "A distinctive small image spot (corner, edge) that can be found again in other images"],
          ["Gazebo", "Free robot simulator: physics, sensors, rendering"],
          ["Ground truth", "The true answer (here: the exact position from motion capture or the simulator)"],
          ["gz-transport, topic", "Gazebo's messaging; a topic is a named channel such as /scan"],
          ["Intrinsics", "Camera numbers fx, fy, cx, cy that describe how 3D maps to pixels"],
          ["Keyframe", "A frame kept in the SLAM map because it shows something new"],
          ["LightGlue", "Learned feature matcher (2023) that matches all keypoints of two images jointly"],
          ["Lidar", "A sensor that measures distances with laser beams in many directions"],
          ["Loop closure", "Recognising a previously visited place and correcting drift"],
          ["Marching cubes", "Turning a voxel distance field into a triangle mesh"],
          ["Mesh", "A 3D surface made of triangles"],
          ["Morton (Z-order) curve", "An ordering that keeps nearby 3D points near each other in a list"],
          ["Map merging", "Joining two separate SLAM maps once a shared place is recognised"],
          ["Pose graph", "Keyframe poses linked by measured relative poses; optimized to remove drift"],
          ["Pose", "Position + orientation (6 numbers)"],
          ["PnP", "Perspective-n-Point: computing the camera pose from known 3D points seen in an image"],
          ["Pure pursuit", "Steering towards a point a fixed distance ahead on the route"],
          ["RANSAC", "Fitting a model while ignoring wrong data, by trying many random small samples"],
          ["Reprojection error", "Pixel distance between where a 3D point should appear and where it was seen"],
          ["RGB-D", "Colour (RGB) plus depth (D) camera"],
          ["SDF", "Simulation Description Format: Gazebo's XML files for worlds and models"],
          ["SLAM", "Simultaneous Localization And Mapping"],
          ["SuperPoint", "Learned keypoint detector and descriptor (2018), a small CNN"],
          ["Texture / UV", "An image wrapped onto a mesh; UV says which image point goes on which surface point"],
          ["TSDF", "Truncated Signed Distance Function: voxels storing distance to the nearest surface"],
          ["Umeyama alignment", "Best rotation, shift (and scale) to line up two point sets"],
          ["Voxel", "A tiny cube of 3D space, like a 3D pixel"]]
    st += [H1("Glossary"), *table(gl, [0.27, 0.73], compact=True)]
    st = glue_headings(st)

    doc = Doc(str(OUT), pagesize=A4, leftMargin=MARGIN, rightMargin=MARGIN, topMargin=18 * mm,
              bottomMargin=16 * mm, title="Visual SLAM to Autonomous Navigation", author="Azad Kumar Jha",
              subject="Project guide: SLAM, digital twin, Gazebo, navigation")
    doc.h1_count = 0
    doc.multiBuild(st, onFirstPage=on_page, onLaterPages=on_page)
    print("wrote", OUT)


if __name__ == "__main__":
    build()
