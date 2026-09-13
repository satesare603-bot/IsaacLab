# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Save isolated body-offset samples and observe discrete mesh surfaces [m]."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
import render_hand_fingertip_concepts_v01 as base  # noqa: E402


def _snapshot():
    result = base._snapshot()
    for scene, objects in result.items():
        for name, row in objects.items():
            obj = bpy.data.scenes[scene].objects[name]
            row["face_sha256"] = hashlib.sha256(
                json.dumps([list(poly.vertices) for poly in obj.data.polygons]).encode()
            ).hexdigest()
            row["hide_render"] = obj.hide_render
            row["hide_viewport"] = obj.hide_viewport
    return result


def _geometry(obj):
    points = [obj.matrix_world @ vertex.co for vertex in obj.data.vertices]
    faces = [tuple(poly.vertices) for poly in obj.data.polygons]
    lower = tuple(min(p[a] for p in points) for a in range(3))
    upper = tuple(max(p[a] for p in points) for a in range(3))
    return BVHTree.FromPolygons(points, faces, all_triangles=True, epsilon=0.0), lower, upper


def _pairs(scene):
    shapes = {obj.name: _geometry(obj) for obj in scene.objects if obj.type == "MESH"}
    hands = [obj for obj in scene.objects if obj.get("role") in ("hardware", "insert")]
    targets = [obj for obj in scene.objects if obj.get("role") == "target"]
    carriers = [obj for obj in hands if obj.name.endswith("_carrier")]
    hardware = [obj for obj in hands if obj.get("role") == "hardware"]
    left = [obj for obj in hands if obj.get("role") == "insert" and "__left_left_" in obj.name]
    right = [obj for obj in hands if obj.get("role") == "insert" and "__left_right_" in obj.name]
    groups = {
        "hand_vs_reference_context": [(h, t) for h in hands for t in targets],
        "carrier_vs_hardware_including_mounts": [(c, h) for c in carriers for h in hardware],
        "left_vs_right_inserts": [(first, second) for first in left for second in right],
    }
    result = {}
    for label, pairs in groups.items():
        overlaps = []
        for first, second in pairs:
            a, a_lower, a_upper = shapes[first.name]
            b, b_lower, b_upper = shapes[second.name]
            if any(a_upper[i] < b_lower[i] or b_upper[i] < a_lower[i] for i in range(3)):
                continue
            found = a.overlap(b)
            if found:
                overlaps.append(
                    {
                        "first": first.name.split("__", 1)[1],
                        "second": second.name.split("__", 1)[1],
                        "triangle_pairs": len(found),
                    }
                )
        result[label] = {"object_pairs_considered": len(pairs), "surface_overlap_pairs": overlaps}
    return result


def main() -> None:
    """Write a new static Blender model and verify its saved mesh data."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :])
    target = args.directory / "hand_body_setback_v01.blend"
    if target.exists():
        raise FileExistsError(target)
    source = args.directory / "hand_body_setback_meshes_v01.json"
    source_sha = hashlib.sha256(source.read_bytes()).hexdigest()
    payload = json.loads(source.read_text())
    bpy.ops.wm.read_factory_settings(use_empty=True)
    initial = bpy.context.scene
    base._scenes(payload)
    for scene in bpy.data.scenes:
        if scene == initial:
            continue
        focus = Vector((0, 0.048, 0.060))
        scene.camera.location = (0.205, -0.26, 0.19)
        scene.camera.rotation_euler = (focus - scene.camera.location).to_track_quat("-Z", "Y").to_euler()
        scene.camera.data.ortho_scale = 0.290
    bpy.context.window.scene = bpy.data.scenes["D30_near"]
    bpy.data.scenes.remove(initial)
    before = _snapshot()
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(target))
    bpy.ops.wm.open_mainfile(filepath=str(target))
    after = _snapshot()
    if before != after:
        (args.directory / "setback_native_readback_difference.json").write_text(
            json.dumps({"before": before, "after": after}, indent=2) + "\n"
        )
        raise AssertionError("Static setback model changed on readback")
    pairs = {name: _pairs(bpy.data.scenes[name]) for name in sorted(after)}
    for name, groups in pairs.items():
        state = name.split("_", 1)[1]
        for label, group in groups.items():
            baseline = {(row["first"], row["second"]) for row in pairs[f"D00_{state}"][label]["surface_overlap_pairs"]}
            group["additional_object_overlap_pairs_vs_D00"] = [
                row for row in group["surface_overlap_pairs"] if (row["first"], row["second"]) not in baseline
            ]
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_sha
    report = {
        "native_sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
        "source_mesh_payload_sha256": source_sha,
        "native_readback_identical": True,
        "readback_fields": ["world matrices", "vertex SHA", "face SHA", "mesh counts", "visibility"],
        "mesh_counts": {name: len(records) for name, records in after.items()},
        "surface_observations": pairs,
        "method": "Blender BVHTree triangle surfaces, epsilon=0; AABB rejection; same groups for D00/D30/D60",
        "scope": "Three placements and four discrete poses; includes mounting pairs rather than hiding them",
        "limitations": (
            "No signed volume, solid containment, full swept motion, camera, arm or physical validity verdict"
        ),
        "comparison_cylinders": "Shown in HTML only; absent from native and GLB",
        "video_created": False,
        "physical_acceptance_verdict": None,
    }
    (args.directory / "hand_body_setback_native_v01.json").write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps(
            {
                name: {group: len(row["surface_overlap_pairs"]) for group, row in groups.items()}
                for name, groups in pairs.items()
            },
            indent=2,
        ),
        flush=True,
    )
    print("HAND_BODY_SETBACK_NATIVE_DONE", flush=True)


if __name__ == "__main__":
    main()
