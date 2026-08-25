#!/usr/bin/env python3
"""Render the output of g1_skeleton_dump as an animated GIF.

    ros2 run g1_model g1_skeleton_dump <urdf> 48 | tools/render_skeleton.py docs/g1_squat.gif

Two orthographic views (front and side) of the kinematic tree, with the whole-body
centre of mass and its ground projection. Each frame is shifted vertically so the
lowest foot sits on the ground line, which is what makes the squat read as a squat:
the model itself is expressed in the pelvis frame.
"""

import sys
from PIL import Image, ImageDraw

# Frames that carry no structure worth drawing (sensors, covers, logo).
# Frames whose origin carries no structure: sensors and cosmetic links whose
# geometry lives entirely in the mesh offset (head_link's origin is the pelvis).
SKIP = {
    "imu_in_pelvis", "imu_in_torso", "pelvis_contour_link", "logo_link",
    "d435_link", "mid360_link", "waist_support_link", "head_link",
}

W, H = 340, 420          # per view
SCALE = 210              # pixels per metre
DURATION_MS = 60

BG = (250, 250, 250)
GROUND = (205, 205, 205)
LEFT = (26, 115, 200)
RIGHT = (214, 106, 20)
SPINE = (60, 60, 60)
COM = (200, 40, 60)
LABEL = (120, 120, 120)


def parse(stream):
    edges, frames = [], []
    links, com = {}, None
    for line in stream:
        kind, _, rest = line.strip().partition(" ")
        if kind == "edge":
            parent, child = rest.split()
            edges.append((parent, child))
        elif kind == "frame":
            if links:
                frames.append((links, com))
            links, com = {}, None
        elif kind == "link":
            name, x, y, z = rest.split()
            links[name] = (float(x), float(y), float(z))
        elif kind == "com":
            com = tuple(float(v) for v in rest.split())
    if links:
        frames.append((links, com))
    return edges, frames


def side_color(name):
    if name.startswith("left"):
        return LEFT
    if name.startswith("right"):
        return RIGHT
    return SPINE


def draw_view(draw, links, com, edges, ox, horizontal, flip, ground_z):
    """Draw one orthographic view; `horizontal` picks the in-plane axis (1=y, 0=x).

    `flip` mirrors that axis: the front view looks at the robot from +x, so its
    left side (+y) belongs on the left of the image.
    """
    cx, base_y = ox + W // 2, H - 60

    def project(p):
        return (cx + flip * p[horizontal] * SCALE, base_y - (p[2] - ground_z) * SCALE)

    draw.line([(ox + 20, base_y), (ox + W - 20, base_y)], fill=GROUND, width=3)

    for parent, child in edges:
        if parent in SKIP or child in SKIP or parent not in links or child not in links:
            continue
        a, b = project(links[parent]), project(links[child])
        draw.line([a, b], fill=side_color(child), width=5)

    for name, p in links.items():
        if name in SKIP:
            continue
        x, y = project(p)
        draw.ellipse([x - 3, y - 3, x + 3, y + 3], fill=side_color(name))

    if com is not None:
        x, y = project(com)
        draw.line([(x, y), (x, base_y)], fill=COM, width=1)
        draw.ellipse([x - 6, y - 6, x + 6, y + 6], outline=COM, width=3)
        draw.ellipse([x - 3, base_y - 3, x + 3, base_y + 3], fill=COM)


def render(edges, frames, out_path):
    images = []
    for links, com in frames:
        drawable = {n: p for n, p in links.items() if n not in SKIP}
        ground_z = min(p[2] for p in drawable.values())

        image = Image.new("RGB", (2 * W, H), BG)
        draw = ImageDraw.Draw(image)
        draw_view(draw, links, com, edges, 0, 1, -1, ground_z)
        draw_view(draw, links, com, edges, W, 0, 1, ground_z)
        draw.line([(W, 24), (W, H - 24)], fill=GROUND, width=1)
        draw.text((18, 16), "front  (y-z)", fill=LABEL)
        draw.text((W + 18, 16), "side  (x-z)", fill=LABEL)
        draw.text((18, H - 26), "g1_model - FK + centre of mass", fill=LABEL)
        images.append(image.quantize(colors=32))

    images[0].save(
        out_path, save_all=True, append_images=images[1:], duration=DURATION_MS,
        loop=0, optimize=True,
    )


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: g1_skeleton_dump ... | render_skeleton.py <out.gif>")
    edges, frames = parse(sys.stdin)
    if not frames:
        sys.exit("render_skeleton.py: no frames on stdin")
    render(edges, frames, sys.argv[1])
    print(f"wrote {sys.argv[1]} ({len(frames)} frames)")


if __name__ == "__main__":
    main()
