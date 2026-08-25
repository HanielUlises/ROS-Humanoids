#!/usr/bin/env python3
"""Render a screenshot of the G1 standing in MuJoCo.

    MUJOCO_GL=egl tools/render_mujoco.py ../description/mjcf/g1_29dof.xml docs/g1_mujoco.png

Renders the nominal standing pose offscreen: no display, no simulator window
and no controller. The MJCF has a free-floating base and no actuator commands,
so the pose is held rather than simulated.
"""

import os
import sys
from xml.etree import ElementTree

import mujoco
import numpy as np
from PIL import Image

WIDTH, HEIGHT = 1100, 760


def find_meshes(start):
    """First directory at or above `start` that holds a meshes/ folder."""
    directory = start
    while not os.path.isdir(os.path.join(directory, "meshes")):
        parent = os.path.dirname(directory)
        if parent == directory:
            sys.exit(f"render_mujoco.py: no meshes/ directory at or above {start}")
        directory = parent
    return os.path.join(directory, "meshes")


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: render_mujoco.py <scene.xml> <out.png>")
    scene, out_path = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])

    # MuJoCo resolves meshdir relative to the XML, but the description package
    # keeps its meshes one level above mjcf/, so point meshdir at them directly.
    xml = ElementTree.parse(scene)
    meshes = find_meshes(os.path.dirname(scene))
    for compiler in xml.getroot().iter("compiler"):
        compiler.set("meshdir", meshes)

    # The offscreen framebuffer defaults to 640x480, too small for a crisp shot.
    visual = xml.getroot().find("visual") or ElementTree.SubElement(xml.getroot(), "visual")
    glob = visual.find("global")
    if glob is None:
        glob = ElementTree.SubElement(visual, "global")
    glob.set("offwidth", str(WIDTH))
    glob.set("offheight", str(HEIGHT))

    model = mujoco.MjModel.from_xml_string(ElementTree.tostring(xml.getroot(), "unicode"))
    data = mujoco.MjData(model)
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.lookat[:] = [0.0, 0.0, 0.85]
    camera.distance = 2.4
    camera.azimuth = 140
    camera.elevation = -10

    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()

    # 256 colours keeps the screenshot well under a megabyte in the repository.
    image = Image.fromarray(np.asarray(pixels)).quantize(colors=256, dither=Image.FLOYDSTEINBERG)
    image.save(out_path, optimize=True)
    print(f"wrote {out_path} ({WIDTH}x{HEIGHT})")


if __name__ == "__main__":
    main()
