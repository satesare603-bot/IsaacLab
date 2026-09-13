# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Save the source-matched mounting samples and observe their static surfaces [m]."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
import render_hand_fingertip_concepts_v01 as renderer  # noqa: E402
import save_hand_body_setback_v01 as previous  # noqa: E402


def main() -> None:
    """Save eight static scenes and compare mesh data after reloading."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :])
    source = args.directory / "hand_mount_interface_meshes_v02.json"
    target = args.directory / "hand_mount_interface_v02.blend"
    if target.exists():
        raise FileExistsError(target)
    source_sha = hashlib.sha256(source.read_bytes()).hexdigest()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    initial = bpy.context.scene
    renderer._scenes(json.loads(source.read_text()))
    for scene in bpy.data.scenes:
        if scene == initial:
            continue
        focus = Vector((0, 0.048, 0.060))
        scene.camera.location = (0.205, -0.26, 0.19)
        scene.camera.rotation_euler = (focus - scene.camera.location).to_track_quat("-Z", "Y").to_euler()
        scene.camera.data.ortho_scale = 0.290
    bpy.context.window.scene = bpy.data.scenes["D30_near"]
    bpy.data.scenes.remove(initial)
    before = previous._snapshot()
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(target))
    bpy.ops.wm.open_mainfile(filepath=str(target))
    after = previous._snapshot()
    assert before == after, "Saved static mesh data changed"
    pairs = {name: previous._pairs(bpy.data.scenes[name]) for name in sorted(after)}
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_sha
    report = {
        "native_sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
        "source_mesh_payload_sha256": source_sha,
        "native_readback_identical": True,
        "readback_fields": ["world matrices", "vertex SHA", "face SHA", "mesh counts", "visibility"],
        "mesh_counts": {name: len(records) for name, records in after.items()},
        "surface_observations": pairs,
        "method": "Reused Blender BVHTree triangle surfaces, epsilon=0; AABB rejection; mounting pairs included",
        "limitations": "Surface contact is reported, not classified as solid penetration or physical failure",
        "comparison_cylinders": "Shown in HTML only; absent from native and GLB",
        "continuous_motion_checked": False,
        "physical_acceptance_verdict": None,
    }
    (args.directory / "hand_mount_interface_native_v02.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(pairs, indent=2), flush=True)
    print("HAND_MOUNT_NATIVE_DONE", flush=True)


if __name__ == "__main__":
    main()
