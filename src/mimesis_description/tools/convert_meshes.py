#!/usr/bin/env python3
"""Convert Elephant Robotics' myCobot 280 JN Collada meshes into the meshes this package uses.

Why this exists: the original .dae files mix units (two are in millimetres, the rest in metres),
one has a broken material reference, and each needs a hand-tuned visual offset in the URDF.
This script fixes all of that once, reproducibly:

* reads each .dae (ignoring the broken material), applies its unit scale and the visual
  offset from Elephant's URDF, so the output is in metres and already in the link frame;
* writes visual/<link>.stl (full detail) and collision/<link>.stl (convex hull: fast for MoveIt);
* writes urdf/mycobot_280_jn_inertials.yaml: mass, centre of mass and inertia per link,
  computed from the convex hull with uniform density.

The masses below are ESTIMATES. Weigh the real parts and re-run to improve them.

Usage (not part of the build; needs: pip install trimesh pycollada scipy numpy):
    python3 tools/convert_meshes.py <path to mycobot_ros2>/mycobot_description/urdf/mycobot_280_jn
Source: https://github.com/elephantrobotics/mycobot_ros2 (branch humble), BSD-2-Clause.
"""

import argparse
import io
import pathlib
import re

import collada
import numpy as np
import trimesh
from trimesh.transformations import euler_matrix, translation_matrix

# link name -> (source mesh, visual xyz, visual rpy), copied from Elephant's mycobot_280_jn.urdf
LINKS = {
    "base_link": ("joint1_jet.dae", (0.0, 0.0, 0.0), (0.0, 0.0, 3.14159)),
    "link1": ("joint2.dae", (0.0, 0.0, -0.06096), (0.0, 0.0, -1.5708)),
    "link2": ("joint3.dae", (0.0, 0.0, 0.03256), (0.0, -1.5708, 0.0)),
    "link3": ("joint4.dae", (0.0, 0.0, 0.03056), (0.0, -1.5708, 0.0)),
    "link4": ("joint5.dae", (0.0, 0.0, -0.03356), (0.0, -1.5708, 1.5708)),
    "link5": ("joint6.dae", (0.0, 0.0, -0.038), (0.0, 0.0, 0.0)),
    "link6": ("joint7.dae", (0.0, 0.0, -0.012), (0.0, 0.0, 0.0)),
}

# ESTIMATES in kg. The arm's moving mass is split between links by convex-hull volume.
BASE_MASS = 0.60
ARM_MASS = 0.60


def load_collada(path):
    """Return all triangles of a .dae as one trimesh, in metres, with node transforms applied."""
    # Materials are not needed for STL, and joint5.dae references one that does not exist,
    # which makes the whole scene graph fail to load. Drop every material binding first.
    text = re.sub(
        r"<bind_material>.*?</bind_material>", "", path.read_text(), flags=re.DOTALL
    )
    dae = collada.Collada(
        io.BytesIO(text.encode()),
        ignore=[collada.common.DaeBrokenRefError, collada.common.DaeUnsupportedError],
    )
    scale = dae.assetInfo.unitmeter or 1.0
    parts = []
    for geometry in dae.scene.objects("geometry"):
        for primitive in geometry.primitives():
            if isinstance(primitive, collada.polylist.BoundPolylist):
                primitive = primitive.triangleset()
            if not isinstance(primitive, collada.triangleset.BoundTriangleSet):
                continue
            if primitive.vertex is None or len(primitive.vertex_index) == 0:
                continue
            parts.append(
                trimesh.Trimesh(
                    vertices=primitive.vertex,
                    faces=primitive.vertex_index,
                    process=False,
                )
            )
    mesh = trimesh.util.concatenate(parts)
    mesh.apply_scale(scale)
    mesh.merge_vertices()
    return mesh


def to_link_frame(mesh, xyz, rpy):
    transform = translation_matrix(xyz) @ euler_matrix(*rpy, axes="sxyz")
    mesh.apply_transform(transform)
    return mesh


def inertial_yaml(name, hull, mass):
    hull = hull.copy()
    hull.density = mass / hull.volume
    com = hull.center_mass
    i = hull.moment_inertia  # about the centre of mass, link-frame axes
    return (
        f"{name}:\n"
        f"  mass: {mass:.4f}\n"
        f'  com: "{com[0]:.5f} {com[1]:.5f} {com[2]:.5f}"\n'
        f"  ixx: {i[0, 0]:.4e}\n  ixy: {i[0, 1]:.4e}\n  ixz: {i[0, 2]:.4e}\n"
        f"  iyy: {i[1, 1]:.4e}\n  iyz: {i[1, 2]:.4e}\n  izz: {i[2, 2]:.4e}\n"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "source", type=pathlib.Path, help="folder with Elephant's .dae files"
    )
    args = parser.parse_args()

    package = pathlib.Path(__file__).resolve().parent.parent
    out = package / "meshes" / "mycobot_280_jn"
    (out / "visual").mkdir(parents=True, exist_ok=True)
    (out / "collision").mkdir(parents=True, exist_ok=True)

    hulls = {}
    for link, (source, xyz, rpy) in LINKS.items():
        mesh = to_link_frame(load_collada(args.source / source), xyz, rpy)
        hull = mesh.convex_hull
        mesh.export(out / "visual" / f"{link}.stl")
        hull.export(out / "collision" / f"{link}.stl")
        hulls[link] = hull
        size_mm = np.round(mesh.extents * 1000.0, 1)
        print(f"{link:10s} {len(mesh.faces):7d} faces, size {size_mm} mm")

    arm_volume = sum(h.volume for name, h in hulls.items() if name != "base_link")
    text = [
        "# GENERATED by tools/convert_meshes.py. Do not edit by hand: change the script.\n",
        "# Masses (kg) are estimates; centre of mass (m) and inertia (kg*m^2) assume uniform\n",
        "# density over each link's convex hull, in the link frame.\n",
    ]
    for link, hull in hulls.items():
        mass = BASE_MASS if link == "base_link" else ARM_MASS * hull.volume / arm_volume
        text.append(inertial_yaml(link, hull, mass))
    (package / "urdf" / "mycobot_280_jn_inertials.yaml").write_text("".join(text))


if __name__ == "__main__":
    main()
