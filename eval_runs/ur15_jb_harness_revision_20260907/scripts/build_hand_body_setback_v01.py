# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Compare rearward body offsets with the original terminal grasp location [m]."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
from datetime import datetime
from pathlib import Path

import build_hand_fingertip_concepts_v01 as base
import numpy as np
import probe_hand_tool_access_v01 as probe
import trimesh


def _world(candidate, name, state):
    obj = candidate["objects"][name]
    matrix = np.asarray(candidate["states"][state]["transforms"][name])
    return np.asarray(obj["vertices"]) @ matrix[:3, :3].T + matrix[:3, 3]


def _candidate(original, offset, settings):
    result = copy.deepcopy(original)
    translation = np.array(settings["offset_direction_world"]) * offset
    for state, snapshot in result["states"].items():
        for name, obj in result["objects"].items():
            if obj["category"] not in ("hardware", "insert"):
                continue
            matrix = np.asarray(original["states"][state]["transforms"][name]).copy()
            matrix[:3, 3] += translation
            snapshot["transforms"][name] = matrix.tolist()
    if offset == 0:
        return result
    for name, obj in result["objects"].items():
        if obj["category"] != "insert":
            continue
        world = _world(original, name, "near")
        if name.endswith("_carrier"):
            threshold = settings["carrier_rear_vertex_selection_y_m"]
            interior = (world[:, 1] > settings["carrier_front_y_m"] + 1e-8) & (
                world[:, 1] < settings["carrier_rear_y_m"] - 1e-8
            )
            assert not interior.any(), "Source carrier has a new vertex layer; inspect before extending"
            world[world[:, 1] > threshold] += translation
        else:
            assert name.endswith("_edge_contour"), f"Unmapped insert: {name}"
        # Fix the contact in world space, then attach its new local shape to
        # the translated original distal link. All four original joint states remain.
        matrix = np.asarray(result["states"]["near"]["transforms"][name])
        obj["vertices"] = ((world - matrix[:3, 3]) @ matrix[:3, :3]).tolist()
    return result


def _preservation(original, candidate, offset, settings):
    translation = np.array(settings["offset_direction_world"]) * offset
    hardware_delta, contact_delta, mounting_delta = [], [], []
    carriers = []
    for name, obj in candidate["objects"].items():
        category = obj["category"]
        original_obj = original["objects"][name]
        if category in ("target", "guide", "hardware"):
            assert obj == original_obj, f"Original mesh changed: {name}"
        for state, snapshot in candidate["states"].items():
            assert snapshot["joint_q_rad"] == original["states"][state]["joint_q_rad"]
            difference = _world(candidate, name, state) - _world(original, name, state)
            if category in ("target", "guide"):
                assert np.array_equal(difference, np.zeros_like(difference)), (name, state)
            elif category == "hardware":
                hardware_delta.append(float(np.max(np.abs(difference - translation))))
            elif name.endswith("_edge_contour"):
                contact_delta.append(float(np.max(np.abs(difference))))
            elif name.endswith("_carrier"):
                near = _world(original, name, "near")
                rear = near[:, 1] > settings["carrier_rear_vertex_selection_y_m"]
                mounting_delta.append(float(np.max(np.abs(difference[rear] - translation))))
        if name.endswith("_carrier"):
            mesh = trimesh.Trimesh(obj["vertices"], obj["faces"], process=False)
            assert mesh.is_watertight and mesh.is_winding_consistent, name
            near = _world(candidate, name, "near")
            carriers.append(
                {
                    "object": name,
                    "world_bounds_near_m": [near.min(axis=0).tolist(), near.max(axis=0).tolist()],
                    "closed_surface": bool(mesh.is_watertight),
                    "volume_m3": abs(float(mesh.volume)),
                }
            )
    errors = {
        "hardware_world_translation_max_error_m": max(hardware_delta),
        "edge_contact_world_preservation_max_error_m": max(contact_delta),
        "carrier_mount_translation_max_error_m": max(mounting_delta),
    }
    assert max(errors.values()) < 1e-10, errors
    return {
        **errors,
        "original_hardware_vertices_faces_colors_unchanged": True,
        "targets_and_saved_joint_angles_unchanged": True,
        "carrier_faces_preserved": all(
            obj["faces"] == original["objects"][name]["faces"]
            for name, obj in candidate["objects"].items()
            if name.endswith("_carrier")
        ),
        "carriers": carriers,
    }


def _measure(candidate, settings, seat):
    states = {}
    lower = np.asarray(settings["tool_lower_z_above_seat_m"]) + seat
    upper = np.asarray(settings["tool_upper_z_above_seat_m"]) + seat
    for state in candidate["states"]:
        triangles, names, _ = probe._world_hand(candidate, state, [0, 0])
        record = {}
        for label, band in (("lower", lower), ("upper", upper), ("whole", [lower[0], upper[1]])):
            groups = {}
            for category in ("all", "hardware", "insert"):
                selected = np.array(
                    [category == "all" or candidate["objects"][name]["category"] == category for name in names]
                )
                groups[category] = probe._slab_nearest(
                    triangles[selected], [name for name, take in zip(names, selected, strict=True) if take], band
                )
            record[label] = groups
        states[state] = record
    return states


def _report_text(report):
    lines = [
        "# ハンド本体の後退比較 v01",
        "",
        "端子を保持する濃色の先端輪郭を残し、黒い駆動本体・連動リンクを後方へ離す比較です。",
        "端子・工具軸・先端の接触候補位置を固定し、青い取付部の後方部分だけを延長しました。",
        "実物の製作仕様・採用後退量ではありません。上押さえは加えていません。",
        "",
        "## 近接姿勢での比較",
        "",
        "| 本体の後退量 | 青い部材の前後寸法 | 取付中心から前端 | 上部帯の本体までの最短半径 | φ40上部円筒との差 |",
        "|---|---:|---:|---:|---:|",
    ]
    for name, offset in report["settings"]["setbacks_m"].items():
        row = report["candidates"][name]
        radius = row["tool_surface_observations"]["near"]["upper"]["hardware"]["radius_to_surface_m"] * 1000
        lines.append(
            f"| {offset * 1000:.0f} mm | {33 + offset * 1000:.0f} mm | {23 + offset * 1000:.0f} mm | "
            f"{radius:.3f} mm | {radius - 20:.3f} mm |"
        )
    lines += [
        "",
        "工具の下側はφ12／接続面から3.8〜53.8 mm、上側はφ40／53.8〜188.877 mmの比較円筒です。",
        "これらは実ソケット・締付ユニットの仕様ではありません。ページで上側の径を変更できます。",
        "後退させても、端子近くの青い部材は保持位置に残ります。下側の空間が後退量だけ増えるわけではありません。",
        "差は工具軸から三角形表面までの最短水平距離から比較半径を引いた値です。力学や実工具全体の合否ではありません。",
        "",
        "## 保ったものと変えたもの",
        "",
        "元の黒いハンド10メッシュの頂点・面・色、連動関節角度は不変。world位置へ一様な+Y後退を加えました。",
        "先端の濃色輪郭は形状を保ち、全4保存姿勢で元のworld位置との数値差を記録しています。",
        "青い部材の前方Y=3〜12 mmを保ち、Y=16 mm以降の取付側を後退。間の側方梁を延長します。",
        "側方梁の断面4×6 mmは元の比較値のままで、材料・たわみ・ねじ配置・締結反力への適合は未定です。",
        "",
        "## 公式仕様との関係",
        "",
        "[Robotiq公式 §6.1.2・§6.2.1・§6.2.4](" + report["settings"]["official_reference"]["url"] + ")を参照しました。",
        "固定末節を加工せず、取り付ける交換先端を変更する方式です。公式の先端寸法制約は取付基準からの値で、",
        "今回の30/60 mm後退を承認する値ではありません。標準先端の把持力や精度も延長品へ転用しません。",
        "長さを増やすほど工具本体の空間は広がりますが、指の曲げ負荷・たわみとの両立が必要です。",
        "",
        "参照端子は圧着前の保存Klauke_6R6サンプルで、Ampere実部品への同定は未了です。",
        "CAD全長47 mmと公開表l=37 mmの寸法区間対応は未解決。仮線径14 mmも採用値ではありません。",
        "",
        "## 観測の範囲",
        "",
        "数値観測は3配置×4保存姿勢です。保存読戻し、参考物との三角形表面交差は補助的な観測です。",
        "全連続開閉・カメラ・腕・相手ハンド・実工具・ケースは含まず、正式な物理妥当性判定は行いません。",
        "工程動画・アーム軌道は停止したままです。上押さえを使わず保持したまま締結する方針を維持します。",
        "",
    ]
    return "\n".join(lines)


def build(source: Path, config_path: Path, output: Path) -> dict:
    """Create new static hand offsets [m] and export their geometry observations."""
    settings = json.loads(config_path.read_text())
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    assert before == settings["source_mesh_sha256"], "Source SHA mismatch"
    payload = json.loads(source.read_text())
    original = payload["candidates"][settings["source_candidate"]]
    result = {"config": copy.deepcopy(payload["config"]), "candidates": {}}
    records = {}
    output.mkdir(parents=True, exist_ok=False)
    for name, offset in settings["setbacks_m"].items():
        candidate = _candidate(original, offset, settings)
        preserved = _preservation(original, candidate, offset, settings)
        observed = _measure(candidate, settings, payload["config"]["target"]["seat_z_m"])
        exported = base._export_glb(candidate, output / f"hand_body_setback_{name}_v01.glb")
        assert exported["mesh_count"] == exported["readback_mesh_count"]
        assert exported["readback_bounds_max_difference_m"] < 1e-7
        result["candidates"][name] = candidate
        records[name] = {
            "setback_m": offset,
            "preservation": preserved,
            "tool_surface_observations": observed,
            "glb": exported,
        }
        radius = observed["near"]["upper"]["hardware"]["radius_to_surface_m"]
        print(f"SETBACK_OBSERVED {name}: upper hardware radius={radius:.9f} m", flush=True)
    assert hashlib.sha256(source.read_bytes()).hexdigest() == before
    report = {
        "recorded_at": datetime.now().astimezone().isoformat(),
        "settings": settings,
        "source_sha256": before,
        "source_file_unchanged": True,
        "config_sha256": base._digest(config_path),
        "candidates": records,
        "source_meshes_per_hand": len(original["objects"]),
        "method": "Reused Z-slab clipped surface projection; full source hardware and carrier triangles, float64",
        "numerical_preservation_check_m": 1e-10,
        "glb_bounds_readback_check_m": 1e-7,
        "numerical_tolerances_are_acceptance_criteria": False,
        "physical_acceptance_verdict": None,
    }
    packed = json.dumps(result, separators=(",", ":"))
    (output / "hand_body_setback_meshes_v01.json").write_text(packed + "\n")
    (output / "hand_body_setback_observations_v01.json").write_text(json.dumps(report, indent=2) + "\n")
    template = Path(__file__).with_name("hand_body_setback_viewer_v01.html")
    page = template.read_text().replace("__MESH_PAYLOAD__", packed)
    page = page.replace("__OBSERVATION_PAYLOAD__", json.dumps(report, separators=(",", ":")))
    (output / "ハンド本体の後退比較_v01.html").write_text(page)
    (output / "ハンド本体の後退比較_v01.md").write_text(_report_text(report))
    (output / "inputs").mkdir()
    shutil.copy2(source, output / "inputs/hand_terminal_meshes_v01.json")
    (output / "data").mkdir()
    shutil.copy2(config_path, output / "data" / config_path.name)
    (output / "scripts").mkdir()
    for path in (Path(__file__), Path(base.__file__), Path(probe.__file__), template):
        shutil.copy2(path, output / "scripts" / path.name)
    return report


def main() -> None:
    """Parse the source and fresh output paths for the static comparison."""
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "config", "output_directory"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    args = parser.parse_args()
    build(args.source, args.config, args.output_directory)
    print("HAND_BODY_SETBACK_DONE", flush=True)


if __name__ == "__main__":
    main()
