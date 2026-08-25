#!/usr/bin/env python3
"""Render a screenshot of a robot standing in MuJoCo.

    MUJOCO_GL=egl tools/render_mujoco.py <scene.xml> <out.png>

Renders the nominal pose offscreen, without a display or a simulator window.
The MJCF is loaded from a temporary copy placed beside the original, so that
includes, mesh directories and other relative paths resolve as they normally
would.
"""

import os
import sys
import tempfile
from xml.etree import ElementTree

import mujoco
import numpy as np
from PIL import Image

WIDTH, HEIGHT = 1100, 760


def meshes_directory(start):
    """First directory at or above `start` that holds a meshes/ folder."""
    directory = start
    while not os.path.isdir(os.path.join(directory, "meshes")):
        parent = os.path.dirname(directory)
        if parent == directory:
            return None
        directory = parent
    return os.path.join(directory, "meshes")


def patched_model(scene):
    """Load the MJCF with a render friendly framebuffer size and headlight."""
    directory = os.path.dirname(scene)
    tree = ElementTree.parse(scene)
    root = tree.getroot()

    meshes = meshes_directory(directory)
    for compiler in root.iter("compiler"):
        if compiler.get("meshdir") and meshes:
            compiler.set("meshdir", meshes)

    visual = root.find("visual")
    if visual is None:
        visual = ElementTree.SubElement(root, "visual")
    settings = visual.find("global")
    if settings is None:
        settings = ElementTree.SubElement(visual, "global")
    settings.set("offwidth", str(WIDTH))
    settings.set("offheight", str(HEIGHT))

    # Dark meshes come out nearly black under the default headlight.
    headlight = visual.find("headlight")
    if headlight is None:
        headlight = ElementTree.SubElement(visual, "headlight")
    headlight.set("diffuse", "1.0 1.0 1.0")
    headlight.set("ambient", "0.6 0.6 0.6")
    headlight.set("specular", "0.2 0.2 0.2")

    handle, path = tempfile.mkstemp(suffix=".xml", dir=directory)
    os.close(handle)
    try:
        tree.write(path)
        return mujoco.MjModel.from_xml_path(path)
    finally:
        os.unlink(path)


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: render_mujoco.py <scene.xml> <out.png>")
    scene, out_path = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])

    model = patched_model(scene)
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

    image = Image.fromarray(np.asarray(pixels)).quantize(colors=256, dither=Image.FLOYDSTEINBERG)
    image.save(out_path, optimize=True)
    print(f"wrote {out_path} ({WIDTH}x{HEIGHT})")


if __name__ == "__main__":
    main()
