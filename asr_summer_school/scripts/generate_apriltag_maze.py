#!/usr/bin/env python3
"""Build hard_maze_apriltag.world: hard_maze_base.world with AprilTags on its walls.

Tag models follow the layout of koide3/gazebo_apriltag (https://github.com/koide3/gazebo_apriltag)
and their textures come from AprilRobotics/apriltag-imgs.

    # regenerate the tag models too (needs opencv + a checkout of apriltag-imgs)
    git clone https://github.com/AprilRobotics/apriltag-imgs.git
    ./generate_apriltag_maze.py --apriltag-imgs apriltag-imgs

    # only re-emit the world, reusing the tag models already in models/
    ./generate_apriltag_maze.py

Tags are placed only on wall faces that front open space, so a robot driving the
maze can actually see them. Every tag carries a distinct tag36h11 id.

NOTE ON SIZE: a tag36h11 image is 10x10 cells, of which the detectable
black-bordered tag is the inner 8x8. The detector's "tag size" is therefore
0.8 * the plate size set here: 0.80 m on the border walls, 0.40 m on the maze
obstacles.
"""
import argparse
import math
import os
import shutil
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC_WORLD = os.path.join(PKG, "worlds", "hard_maze_base.world")
OUT_WORLD = os.path.join(PKG, "worlds", "hard_maze_apriltag.world")
MODELS = os.path.join(PKG, "models")

ARENA = 10.0          # inner face of the border walls, at +-ARENA on both axes

# Sized for turtlebot3_perception: apriltag.yaml declares one tag size (0.16 m) and
# camera.launch.py puts the RealSense at z=0.240 on base_footprint, looking level.

TAG = 0.16            # detectable tag edge, i.e. what apriltag.yaml's `size` must be
PLATE = TAG / 0.8     # 0.20 m -- the tag is the inner 8 of the image's 10 cells
TAG_Z = 0.24          # plate centre at camera height, so tags fill the frame vertically
STANDOFF = 0.006      # push the plate off the wall face to avoid z-fighting
N_TAGS = 12           # a sparse set to go and find, not a saturated arena
MIN_OPEN = 1.0        # only mount where the corridor in front runs at least this far
VIEW_RANGE = 3.0      # a tag needs VIEW_RANGE * PLATE of clear space in front of it,
                      # so the whole plate is framed from beyond the 0.3 m min range
SIDE_MARGIN = 0.10    # ... and that space must clear the plate edges by this much
TEXTURE_PX = 256      # must be a power of two; the source tag image is only 10x10 cells

# ---------------------------------------------------------------- geometry


def load_boxes():
    """Axis-aligned obstacle footprints as (xmin, xmax, ymin, ymax)."""
    world = ET.parse(SRC_WORLD).getroot().find("world")
    obst = next(m for m in world.findall("model") if m.get("name") == "obstacles")
    mp = [float(v) for v in obst.find("pose").text.split()]
    boxes = []
    for link in obst.findall("link"):
        lp = [float(v) for v in link.find("pose").text.split()]
        sx, sy, _ = [float(v) for v in link.find("collision/geometry/box/size").text.split()]
        cx, cy = mp[0] + lp[0], mp[1] + lp[1]
        boxes.append((cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2))
    return boxes


def free(x, y, boxes):
    if not (-ARENA < x < ARENA and -ARENA < y < ARENA):
        return False
    return not any(x0 < x < x1 and y0 < y < y1 for x0, x1, y0, y1 in boxes)


def visible(x, y, n, boxes):
    """True if the whole plate can be seen: the wedge in front of it, as wide as
    the plate and VIEW_RANGE * PLATE deep, must be clear of walls."""
    px, py = -n[1], n[0]                                   # along the wall face
    half = PLATE / 2 + SIDE_MARGIN
    depth = VIEW_RANGE * PLATE
    steps = max(3, int(depth / 0.25))
    for k in range(1, steps + 1):
        d = depth * k / steps
        for t in (-half, 0.0, half):
            if not free(x + n[0] * d + px * t, y + n[1] * d + py * t, boxes):
                return False
    return True


def open_depth(x, y, n, boxes, limit=4.0):
    """How far the corridor in front of a face runs before hitting something."""
    d = 0.0
    while d < limit:
        d += 0.1
        if not free(x + n[0] * d, y + n[1] * d, boxes):
            return d - 0.1
    return limit


def rpy_for(normal):
    """RPY that aims the plate's +Z along `normal` (a horizontal unit vector)
    with the tag image upright and unmirrored.

    The plate is textured on both faces; only the +Z one reads unmirrored.
    """
    # On the plate's +Z face the tag image runs "right" along local +Y and
    # "down" along local +X, so pointing local +X at the ground stands it upright.
    z = (normal[0], normal[1], 0.0)
    x = (0.0, 0.0, -1.0)
    y = (z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0])
    r = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]

    # Decompose as Rz(yaw) Ry(pitch) Rx(roll). Local +X points at the ground, so
    # pitch always lands on the +pi/2 singularity where roll and yaw are
    # degenerate; pin roll to zero and take yaw from the remaining terms.
    pitch = -math.asin(max(-1.0, min(1.0, r[2][0])))
    if abs(r[2][0]) > 1.0 - 1e-9:
        roll = 0.0
        yaw = -math.atan2(r[0][1], r[0][2]) * (1.0 if r[2][0] < 0 else -1.0)
    else:
        roll = math.atan2(r[2][1], r[2][2])
        yaw = math.atan2(r[1][0], r[0][0])
    return roll, pitch, yaw


def faces(boxes):
    """Every mountable wall face: the four border walls plus each obstacle side.

    Yields (normal, offset_of_face, (lo, hi) extent along the face, axis, kind).
    """
    yield ((1.0, 0.0), -ARENA, (-ARENA, ARENA), "y", "border")
    yield ((-1.0, 0.0), ARENA, (-ARENA, ARENA), "y", "border")
    yield ((0.0, -1.0), ARENA, (-ARENA, ARENA), "x", "border")
    yield ((0.0, 1.0), -ARENA, (-ARENA, ARENA), "x", "border")
    for x0, x1, y0, y1 in boxes:
        yield ((1.0, 0.0), x1, (y0, y1), "y", "obstacle")
        yield ((-1.0, 0.0), x0, (y0, y1), "y", "obstacle")
        yield ((0.0, 1.0), y1, (x0, x1), "x", "obstacle")
        yield ((0.0, -1.0), y0, (x0, x1), "x", "obstacle")


def place(boxes, n_tags=N_TAGS):
    """A small, well-spread set of tags.

    Every candidate face already fronts open space; among those we keep the ones
    with real depth in front, then pick by farthest-point sampling so the tags end
    up scattered around the maze and have to be explored for, rather than clustered.
    """
    cands = []
    for n, at, (lo, hi), axis, kind in faces(boxes):
        if hi - lo < PLATE + 0.2:
            continue
        edge = PLATE / 2 + 0.1
        t = lo + edge
        while t <= hi - edge + 1e-9:
            x, y = (at, t) if axis == "y" else (t, at)
            if visible(x, y, n, boxes):
                depth = open_depth(x, y, n, boxes)
                if depth >= MIN_OPEN:
                    cands.append((depth, x, y, n, kind))
            t += 0.25
    if not cands:
        raise SystemExit("no mountable faces found")

    chosen = [max(cands, key=lambda c: c[0])]          # seed: the most open face
    while len(chosen) < min(n_tags, len(cands)):
        best, best_d = None, -1.0
        for c in cands:
            d = min(math.hypot(c[1] - k[1], c[2] - k[2]) for k in chosen)
            if d > best_d:
                best, best_d = c, d
        chosen.append(best)

    chosen.sort(key=lambda c: (-c[2], c[1]))           # north-to-south, for readable ids
    return [(x + n[0] * STANDOFF, y + n[1] * STANDOFF, TAG_Z, n, kind)
            for _d, x, y, n, kind in chosen]


# ---------------------------------------------------------------- emitters

MODEL_SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>1</static>
    <link name='main'>
      <pose>0 0 0 0 0 0</pose>
      <visual name='main_Visual'>
        <geometry>
          <box>
            <size>1.0 1.0 0.01</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>model://{name}/materials/scripts</uri>
            <uri>model://{name}/materials/textures</uri>
            <name>{name}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MODEL_CONFIG = """<?xml version="1.0" ?>
<model>
    <name>{name}</name>
    <version>1.0</version>
    <sdf version="1.6">model.sdf</sdf>
    <author>
        <name>Kenji Koide</name>
        <email>koide@dei.unipd.it</email>
    </author>
    <description>AprilTag {tag} model, after koide3/gazebo_apriltag.</description>
</model>
"""

MATERIAL = """material {name}
{{
  technique
  {{
    pass
    {{
      lighting off
      texture_unit
      {{
        texture {tag}.png
        filtering none none none
        scale 1.0 1.0
      }}
    }}
  }}
}}
"""


def generate_models(ids, imgs_dir):
    import cv2
    for i in ids:
        tag = "tag36_11_%05d" % i
        name = "April" + tag
        src = os.path.join(imgs_dir, "tag36h11", tag + ".png")
        img = cv2.imread(src, 0)
        if img is None:
            raise SystemExit("missing tag image: %s" % src)
        img = cv2.resize(img, (TEXTURE_PX, TEXTURE_PX), interpolation=cv2.INTER_NEAREST)

        root = os.path.join(MODELS, name)
        shutil.rmtree(root, ignore_errors=True)
        os.makedirs(os.path.join(root, "materials", "scripts"))
        os.makedirs(os.path.join(root, "materials", "textures"))
        with open(os.path.join(root, "model.sdf"), "w") as f:
            f.write(MODEL_SDF.format(name=name))
        with open(os.path.join(root, "model.config"), "w") as f:
            f.write(MODEL_CONFIG.format(name=name, tag=tag))
        with open(os.path.join(root, "materials", "scripts", "Apriltag.material"), "w") as f:
            f.write(MATERIAL.format(name=name, tag=tag))
        cv2.imwrite(os.path.join(root, "materials", "textures", tag + ".png"), img)
    print("wrote %d tag models to %s" % (len(ids), MODELS))


def tag_xml(i, spot):
    """One wall-mounted plate. Written inline rather than <include>d so the plate
    carries the size this arena needs instead of the shipped model's 1 m."""
    x, y, z, n, _kind = spot
    roll, pitch, yaw = rpy_for(n)
    name = "Apriltag36_11_%05d" % i
    return """        <model name='{name}'>
            <static>1</static>
            <pose>{x:.3f} {y:.3f} {z:.3f} {r:.6f} {p:.6f} {yw:.6f}</pose>
            <link name='main'>
                <visual name='main_Visual'>
                    <cast_shadows>0</cast_shadows>
                    <geometry>
                        <box>
                            <size>{s} {s} 0.005</size>
                        </box>
                    </geometry>
                    <material>
                        <script>
                            <uri>model://{name}/materials/scripts</uri>
                            <uri>model://{name}/materials/textures</uri>
                            <name>{name}</name>
                        </script>
                    </material>
                </visual>
            </link>
        </model>
""".format(name=name, x=x, y=y, z=z, r=roll, p=pitch, yw=yaw, s=round(PLATE, 4))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--apriltag-imgs", metavar="DIR",
                    help="checkout of AprilRobotics/apriltag-imgs; regenerates the tag models")
    ap.add_argument("--tags", type=int, default=N_TAGS, metavar="N",
                    help="how many tags to scatter through the maze (default %d)" % N_TAGS)
    args = ap.parse_args()

    boxes = load_boxes()
    spots = place(boxes, args.tags)
    ids = list(range(len(spots)))

    if args.apriltag_imgs:
        generate_models(ids, args.apriltag_imgs)

    missing = [i for i in ids if not os.path.isdir(os.path.join(MODELS, "Apriltag36_11_%05d" % i))]
    if missing:
        raise SystemExit("no model for tag ids %s; rerun with --apriltag-imgs" % missing)

    with open(SRC_WORLD) as f:
        base = f.read()
    tags = "".join(tag_xml(i, s) for i, s in zip(ids, spots))
    block = "\n        <!-- AprilTags (tag36h11), generated by scripts/generate_apriltag_maze.py -->\n" + tags
    if base.count("</world>") != 1:
        raise SystemExit("expected exactly one </world> in %s" % SRC_WORLD)
    with open(OUT_WORLD, "w") as f:
        f.write(base.replace("</world>", block + "    </world>"))

    n_border = sum(1 for s in spots if s[4] == "border")
    print("wrote %s\n  %d tags of %.2f m (%.2f m plates) at z=%.2f -- %d on the border walls, %d on the obstacles"
          % (OUT_WORLD, len(spots), TAG, PLATE, TAG_Z, n_border, len(spots) - n_border))


if __name__ == "__main__":
    main()
