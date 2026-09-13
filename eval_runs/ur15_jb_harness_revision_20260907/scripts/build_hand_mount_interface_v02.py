# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Detail the retained 2F-85 mounting interface for static extended fingers [m]."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import numpy as np


def _write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _mount_interface(obj, settings):
    import trimesh

    mesh = trimesh.Trimesh(obj["vertices"], obj["faces"], process=False)
    w = settings["source_mount_face_window_local_m"]
    centers, normals = mesh.triangles_center, mesh.face_normals
    selected = np.flatnonzero(
        (normals[:, 1] < settings["source_plane_normal_y_max"])
        & (centers[:, 2] > w["min_z"])
        & (centers[:, 2] < w["max_z"])
        & (centers[:, 1] > w["min_y"])
    )
    normal = np.average(normals[selected], axis=0, weights=mesh.area_faces[selected])
    normal /= np.linalg.norm(normal)
    points = mesh.triangles[selected].reshape(-1, 3)
    distances = points @ normal
    # The plate starts at the outer supporting envelope of these source facets.
    # No user clearance threshold or deliberate display gap is added.
    support_distance = float(distances.max())
    edges = np.sort(np.concatenate([mesh.faces[selected][:, pair] for pair in ([0, 1], [1, 2], [2, 0])]), axis=1)
    unique, counts = np.unique(edges, axis=0, return_counts=True)
    adjacency = {}
    for first, second in unique[counts == 1]:
        adjacency.setdefault(first, []).append(second)
        adjacency.setdefault(second, []).append(first)
    remaining, boundaries = set(adjacency), []
    while remaining:
        stack, indices = [remaining.pop()], []
        while stack:
            current = stack.pop()
            indices.append(current)
            for following in adjacency[current]:
                if following in remaining:
                    remaining.remove(following)
                    stack.append(following)
        boundary = mesh.vertices[indices]
        boundaries.append(boundary)
    holes = []
    for boundary in boundaries:
        if np.ptp(boundary[:, 2]) >= settings["source_small_loop_span_max_m"]:
            continue
        plane = boundary[:, [0, 2]]
        coefficients = np.linalg.lstsq(np.c_[2 * plane, np.ones(len(plane))], np.square(plane).sum(axis=1), rcond=None)[
            0
        ]
        center_x, center_z = coefficients[:2]
        center_y = (support_distance - normal[0] * center_x - normal[2] * center_z) / normal[1]
        radii = np.linalg.norm(plane - coefficients[:2], axis=1)
        holes.append(
            {
                "center_local_m": [float(center_x), float(center_y), float(center_z)],
                "diameter_projected_m": float(2 * radii.mean()),
                "circle_radius_residual_max_m": float(np.abs(radii - radii.mean()).max()),
                "boundary_vertices": len(boundary),
            }
        )
    holes.sort(key=lambda h: h["center_local_m"][2])
    assert len(holes) == 3, "Unrecognized mount topology; inspect the original source"
    expected = np.array([0.0034, 0.003, 0.0034])
    actual = np.array([h["diameter_projected_m"] for h in holes])
    pitch = float(np.linalg.norm(np.subtract(holes[2]["center_local_m"], holes[0]["center_local_m"])))
    identity_error = max(float(np.abs(expected - actual).max()), abs(pitch - 0.016))
    assert identity_error < settings["measurement_identity_check_m"], "Drawing and source features differ"
    for hole, role in zip(holes, ("screw_lower", "index", "screw_upper"), strict=True):
        hole["role"] = role
    axis_u = np.array([1.0, 0, 0])
    axis_u -= normal * np.dot(normal, axis_u)
    axis_u /= np.linalg.norm(axis_u)
    axis_v = np.cross(normal, axis_u)
    frame = np.eye(4)
    frame[:3, :3] = np.column_stack([axis_u, axis_v, normal])
    frame[:3, 3] = holes[1]["center_local_m"]
    outer = max(boundaries, key=lambda b: np.ptp(b[:, 2]))
    return {
        "holes": holes,
        "screw_pitch_m": pitch,
        "face_width_local_m": float(np.ptp(outer[:, 0])),
        "face_height_local_m": float(np.ptp(outer[:, 2])),
        "face_normal_local": normal.tolist(),
        "support_distance_m": support_distance,
        "source_face_plane_spread_m": float(np.ptp(distances)),
        "source_face_indices": selected.tolist(),
        "drawing_identity_max_error_m": identity_error,
        "mount_frame_local": frame.tolist(),
        "face_z_bounds_local_m": [float(outer[:, 2].min()), float(outer[:, 2].max())],
    }


def _box_geometry(size, center, matrix):
    import trimesh

    mesh = trimesh.creation.box(extents=size)
    mesh.apply_translation(center)
    mesh.apply_transform(matrix)
    return {"vertices": mesh.vertices.tolist(), "faces": mesh.faces.tolist()}


def _prepare(source, settings):
    import build_hand_fingertip_concepts_v01 as base
    import build_hand_terminal_concepts_v01 as terminal

    payload = json.loads(source.read_text())
    first = payload["candidates"]["D30"]["objects"]["hardware_left_right_inner_finger_0"]
    for candidate_name in settings["setbacks_m"]:
        for side in ("left_left", "left_right"):
            assert payload["candidates"][candidate_name]["objects"][f"hardware_{side}_inner_finger_0"] == first
    mount = _mount_interface(first, settings)
    operations = {}
    config = payload["config"]["mechanism"]
    for name, offset in settings["setbacks_m"].items():
        candidate = payload["candidates"][name]
        root_y = config["root_y_m"] + offset
        for side, sign in (("left_left", -1), ("left_right", 1)):
            carrier = f"{side}_carrier"
            matrix = np.asarray(candidate["states"]["near"]["transforms"][carrier])
            inverse = np.linalg.inv(matrix)
            reflect = np.diag([sign, 1, 1, 1])
            front = base._box((0.0015, 0.008, 0.0103), (0.01525, 0.008, -0.00815), "insert")
            rear_y = root_y + settings["rail_rear_from_mount_center_m"]
            section_x, section_z = settings["rail_section_m"]
            rail = base._box(
                (section_x, rear_y - 0.003, section_z),
                (0.016 - section_x / 2, (rear_y + 0.003) / 2, -0.005),
                "insert",
            )
            extended = terminal._box_union([front, rail])
            extended.apply_transform(inverse @ reflect)
            extension = {"vertices": extended.vertices.tolist(), "faces": extended.faces.tolist()}
            plate_frame = np.asarray(mount["mount_frame_local"])
            z_mid = np.mean(mount["face_z_bounds_local_m"])
            center_v = (z_mid - plate_frame[2, 3]) / plate_frame[2, 1]
            thickness = settings["plate_thickness_m"]
            plate = _box_geometry(
                (settings["plate_width_m"], settings["plate_height_m"], thickness),
                (0, center_v, thickness / 2),
                plate_frame,
            )
            operations[f"{name}/{carrier}"] = {
                "parts": [extension, plate],
                "holes": mount["holes"],
                "outward_normal_local": mount["face_normal_local"],
                "plate_thickness_m": thickness,
            }
    return payload, mount, operations


def _boolean_mode(directory):
    import bpy
    from mathutils import Vector

    source = json.loads((directory / "mount_boolean_inputs_v02.json").read_text())
    result = {}
    for name, row in source["operations"].items():
        bpy.ops.wm.read_factory_settings(use_empty=True)
        parts = []
        for index, part in enumerate(row["parts"]):
            mesh = bpy.data.meshes.new(f"part_{index}")
            mesh.from_pydata((np.array(part["vertices"]) * 1000).tolist(), [], part["faces"])
            mesh.update()
            obj = bpy.data.objects.new(mesh.name, mesh)
            bpy.context.collection.objects.link(obj)
            parts.append(obj)
        body = parts[0]
        bpy.context.view_layer.objects.active = body

        def boolean(other, operation):
            modifier = body.modifiers.new(operation, "BOOLEAN")
            modifier.operation = operation
            modifier.solver = "EXACT"
            modifier.object = other
            bpy.ops.object.modifier_apply(modifier=modifier.name)
            bpy.data.objects.remove(other, do_unlink=True)

        for part in parts[1:]:
            boolean(part, "UNION")
        normal = Vector(row["outward_normal_local"])
        thickness = row["plate_thickness_m"] * 1000
        for hole in row["holes"]:
            location = Vector(hole["center_local_m"]) * 1000 + normal * thickness / 2
            bpy.ops.mesh.primitive_cylinder_add(
                vertices=source["settings"]["cylinder_segments"],
                radius=source["settings"]["nominal_mount_bore_diameter_m"] * 500,
                depth=thickness + 4,
                location=location,
                rotation=normal.to_track_quat("Z", "Y").to_euler(),
            )
            cutter = bpy.context.object
            bpy.context.view_layer.objects.active = body
            boolean(cutter, "DIFFERENCE")
        triangulate = body.modifiers.new("triangle_export", "TRIANGULATE")
        bpy.ops.object.modifier_apply(modifier=triangulate.name)
        result[name] = {
            "vertices": [[float(value) / 1000 for value in vertex.co] for vertex in body.data.vertices],
            "faces": [list(poly.vertices) for poly in body.data.polygons],
        }
        print(f"MOUNT_BOOLEAN_OBSERVED {name}: vertices={len(body.data.vertices)}", flush=True)
    _write(directory / "mount_boolean_meshes_v02.json", result)


def _check_and_measure(original, candidate, settings):
    import build_hand_body_setback_v01 as setback
    import trimesh

    carriers = []
    for name, obj in candidate["objects"].items():
        assert candidate["states"] == original["states"], "Saved transforms or joint values changed"
        if not name.endswith("_carrier"):
            assert obj == original["objects"][name], name
            continue
        raw = trimesh.Trimesh(obj["vertices"], obj["faces"], process=False)
        assert raw.is_watertight and raw.is_winding_consistent and raw.euler_number == -4, name
        mesh = trimesh.Trimesh(obj["vertices"], obj["faces"], process=True)
        assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume > 0, name
        components = trimesh.graph.connected_components(
            mesh.face_adjacency, min_len=1, nodes=np.arange(len(mesh.faces))
        )
        assert len(components) == 1, (name, len(components))
        # A genus-three closed plate with three actual through-bores has Euler -4.
        assert mesh.euler_number == -4, (name, mesh.euler_number)
        world = setback._world(candidate, name, "near")
        carriers.append(
            {
                "object": name,
                "world_bounds_near_m": [world.min(0).tolist(), world.max(0).tolist()],
                "closed_surface": True,
                "connected_components": len(components),
                "euler_number": int(mesh.euler_number),
                "raw_edge_topology_closed": bool(raw.is_watertight),
                "raw_zero_area_triangles": int(np.count_nonzero(raw.area_faces == 0)),
                "raw_minimum_triangle_area_m2": float(raw.area_faces.min()),
                "volume_m3": float(mesh.volume),
            }
        )
    return {
        "source_except_carriers_unchanged": True,
        "all_world_matrices_and_joint_values_unchanged": True,
        "carriers": carriers,
    }


def build(args) -> None:
    """Create a new comparison pack with source-matched mounting features [m]."""
    import build_hand_body_setback_v01 as setback
    import build_hand_fingertip_concepts_v01 as base

    settings = json.loads(args.config.read_text())
    source_sha = _sha(args.source)
    assert source_sha == settings["source_sha256"], "Source SHA mismatch"
    payload, mount, operations = _prepare(args.source, settings)
    output = args.output_directory
    output.mkdir(parents=True, exist_ok=False)
    _write(output / "mount_boolean_inputs_v02.json", {"settings": settings, "operations": operations})
    command = [
        str(args.blender),
        "--background",
        "--python-exit-code",
        "1",
        "--python",
        str(Path(__file__).resolve()),
        "--",
        "--boolean_directory",
        str(output),
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    (output / "mount_boolean_stdout.log").write_text(completed.stdout)
    (output / "mount_boolean_stderr.log").write_text(completed.stderr)
    completed.check_returncode()
    meshes = json.loads((output / "mount_boolean_meshes_v02.json").read_text())
    result, records = {"config": payload["config"], "candidates": {}}, {}
    for name, offset in settings["setbacks_m"].items():
        original = payload["candidates"][name]
        candidate = copy.deepcopy(original)
        for side in ("left_left", "left_right"):
            key = f"{side}_carrier"
            candidate["objects"][key].update(meshes[f"{name}/{key}"])
        preserved = _check_and_measure(original, candidate, settings)
        measured = setback._measure(candidate, settings, payload["config"]["target"]["seat_z_m"])
        exported = base._export_glb(candidate, output / f"hand_mount_interface_{name}_v02.glb")
        assert exported["mesh_count"] == exported["readback_mesh_count"]
        assert exported["readback_bounds_max_difference_m"] < 1e-7
        result["candidates"][name] = candidate
        records[name] = {
            "setback_m": offset,
            "preservation": preserved,
            "tool_surface_observations": measured,
            "glb": exported,
        }
    assert _sha(args.source) == source_sha
    report = {
        "recorded_at": datetime.now().astimezone().isoformat(),
        "settings": settings,
        "source_sha256": source_sha,
        "source_file_unchanged": True,
        "mount_interface": mount,
        "candidates": records,
        "physical_acceptance_verdict": None,
    }
    packed = json.dumps(result, separators=(",", ":"))
    (output / "hand_mount_interface_meshes_v02.json").write_text(packed + "\n")
    _write(output / "hand_mount_interface_observations_v02.json", report)
    template = Path(__file__).with_name("hand_mount_interface_viewer_v02.html").read_text()
    template = template.replace("__MESH_PAYLOAD__", packed)
    template = template.replace("__OBSERVATION_PAYLOAD__", json.dumps(report, separators=(",", ":")))
    (output / "延長フィンガの取付比較_v02.html").write_text(template)
    print(json.dumps({"mount_interface": mount, "source_sha256": source_sha}, indent=2), flush=True)
    print("HAND_MOUNT_INTERFACE_DONE", flush=True)


def main() -> None:
    """Parse source, configuration, Blender executable and fresh destination paths."""
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "config", "output_directory", "blender", "boolean_directory"):
        parser.add_argument(f"--{name}", type=Path)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else None)
    if args.boolean_directory:
        _boolean_mode(args.boolean_directory)
    else:
        if not all((args.source, args.config, args.output_directory, args.blender)):
            parser.error("source, config, output_directory and blender are required")
        build(args)


if __name__ == "__main__":
    main()
