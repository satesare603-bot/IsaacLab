# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Observe tool-axis distance to existing static hand surfaces without moving them."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
from datetime import datetime
from pathlib import Path

import numpy as np


def _digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _clip_polygon(points, height, keep_above):
    clipped = []
    previous = points[-1]
    previous_inside = previous[2] >= height if keep_above else previous[2] <= height
    for point in points:
        inside = point[2] >= height if keep_above else point[2] <= height
        if inside != previous_inside:
            fraction = (height - previous[2]) / (point[2] - previous[2])
            crossing = previous + fraction * (point - previous)
            crossing[2] = height
            clipped.append(crossing)
        if inside:
            clipped.append(point)
        previous, previous_inside = point, inside
    return clipped


def _projected_nearest(triangles):
    """Return per-triangle horizontal distance and a 3D witness point."""
    start = triangles
    edge = np.roll(triangles, -1, axis=1) - start
    lengths = np.sum(edge[:, :, :2] ** 2, axis=2)
    fractions = np.divide(
        -np.sum(start[:, :, :2] * edge[:, :, :2], axis=2),
        lengths,
        out=np.zeros_like(lengths),
        where=lengths > 0,
    ).clip(0, 1)
    closest = start + fractions[:, :, None] * edge
    squared = np.sum(closest[:, :, :2] ** 2, axis=2)
    index = squared.argmin(axis=1)
    witness = closest[np.arange(len(triangles)), index].copy()
    distance = np.sqrt(squared[np.arange(len(triangles)), index])

    # A non-degenerate XY projection can contain the axis in its interior.
    origin, v1, v2 = triangles[:, 0], triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    determinant = v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0]
    valid = determinant != 0
    u = np.divide(
        -origin[:, 0] * v2[:, 1] + origin[:, 1] * v2[:, 0],
        determinant,
        out=np.zeros(len(triangles)),
        where=valid,
    )
    v = np.divide(
        -v1[:, 0] * origin[:, 1] + v1[:, 1] * origin[:, 0],
        determinant,
        out=np.zeros(len(triangles)),
        where=valid,
    )
    inside = valid & (u >= 0) & (v >= 0) & (u + v <= 1)
    witness[inside] = origin[inside] + u[inside, None] * v1[inside] + v[inside, None] * v2[inside]
    distance[inside] = 0.0
    return distance, witness


def _slab_nearest(triangles, face_names, limits):
    lower, upper = limits
    z_min, z_max = triangles[:, :, 2].min(axis=1), triangles[:, :, 2].max(axis=1)
    selected = np.flatnonzero((z_min <= upper) & (z_max >= lower))
    pieces, original = [], []
    full = selected[(z_min[selected] >= lower) & (z_max[selected] <= upper)]
    if len(full):
        pieces.append(triangles[full])
        original.extend(full.tolist())
    partial = selected[(z_min[selected] < lower) | (z_max[selected] > upper)]
    clipped_faces = []
    for index in partial:
        polygon = _clip_polygon(list(triangles[index]), lower, True)
        if not polygon:
            continue
        polygon = _clip_polygon(polygon, upper, False)
        for corner in range(1, len(polygon) - 1):
            clipped_faces.append([polygon[0], polygon[corner], polygon[corner + 1]])
            original.append(int(index))
        # Preserve exact tangency as a degenerate surface triangle.
        if 0 < len(polygon) < 3:
            clipped_faces.append([polygon[0], polygon[-1], polygon[-1]])
            original.append(int(index))
    if clipped_faces:
        pieces.append(np.asarray(clipped_faces))
    if not pieces:
        return {"radius_to_surface_m": None, "object": None, "witness_m": None, "source_face_index": None}
    surfaces = np.concatenate(pieces)
    distance, witness = _projected_nearest(surfaces)
    nearest = int(distance.argmin())
    source_index = original[nearest]
    return {
        "radius_to_surface_m": float(distance[nearest]),
        "object": face_names[source_index],
        "witness_m": witness[nearest].tolist(),
        "source_face_index": int(source_index),
    }


def _analytic_checks():
    cases = [
        # Vertical triangle: at z >= 2 the closest edge is x = z, not its x=1 vertex.
        ("clipped_vertical", [[[1, -1, 0], [3, -1, 4], [3, 1, 4]]], [2, 3], 2.0),
        ("projected_axis_inside", [[[-1, -1, 1], [1, -1, 2], [0, 1, 3]]], [0, 4], 0.0),
        ("degenerate_xy_line", [[[2, -1, 0], [2, 1, 0], [2, 0, 4]]], [1, 2], 2.0),
        ("boundary_tangency", [[[2, 0, 2], [3, -1, 3], [3, 1, 3]]], [0, 2], 2.0),
        ("outside_slab", [[[2, 0, 2], [3, -1, 3], [3, 1, 3]]], [4, 5], None),
    ]
    results = []
    for name, coordinates, band, expected in cases:
        triangles = np.asarray(coordinates, dtype=float)
        result = _slab_nearest(triangles, [name], band)
        measured = result["radius_to_surface_m"]
        assert measured is None if expected is None else abs(measured - expected) < 1e-12, (name, result)
        if result["witness_m"] is not None:
            assert band[0] <= result["witness_m"][2] <= band[1]
        results.append({"case": name, "expected": expected, "measured": measured})
    return results


def _world_hand(candidate, state, axis):
    surfaces, names, object_rows = [], [], []
    for name, obj in candidate["objects"].items():
        if obj["category"] not in ("hardware", "insert"):
            continue
        matrix = np.asarray(candidate["states"][state]["transforms"][name])
        world = np.asarray(obj["vertices"]) @ matrix[:3, :3].T + matrix[:3, 3]
        world[:, :2] -= axis
        triangles = world[np.asarray(obj["faces"])]
        surfaces.append(triangles)
        names.extend([name] * len(triangles))
        object_rows.append({"object": name, "category": obj["category"], "triangles": len(triangles)})
    return np.concatenate(surfaces), names, object_rows


def _compare(row, diameters):
    radius = row["radius_to_surface_m"]
    row["comparison_diameter_to_signed_difference_m"] = [
        {"diameter_m": diameter, "difference_m": None if radius is None else radius - diameter / 2}
        for diameter in diameters
    ]
    return row


def _observe(payload, settings):
    seat = payload["config"]["target"]["seat_z_m"]
    previous = payload["config"]["illustrative_surroundings"]
    finite_band = np.asarray(previous["tool_z_above_seat_range_m"])
    geometry = {}
    maximum = 0.0
    for name, candidate in payload["candidates"].items():
        for state in candidate["states"]:
            item = _world_hand(candidate, state, settings["axis_xy_m"])
            geometry[name, state] = item
            maximum = max(maximum, float(item[0][:, :, 2].max()) - seat)
    step = settings["profile_bin_height_m"]
    end = math.ceil(maximum / step) * step
    boundaries = np.arange(0, end + step / 2, step)
    result = {}
    for (name, state), (triangles, names, objects) in geometry.items():
        profile = []
        for lower, upper in zip(boundaries[:-1], boundaries[1:], strict=True):
            row = _slab_nearest(triangles, names, [lower + seat, upper + seat])
            row["z_above_seat_range_m"] = [float(lower), float(upper)]
            profile.append(row)
        row = _slab_nearest(triangles, names, finite_band + seat)
        row["z_above_seat_range_m"] = finite_band.tolist()
        _compare(row, settings["comparison_diameters_m"])
        extended = _slab_nearest(triangles, names, [finite_band[0] + seat, maximum + seat])
        extended["z_above_seat_range_m"] = [float(finite_band[0]), maximum]
        _compare(extended, settings["comparison_diameters_m"])
        result.setdefault(name, {})[state] = {
            "objects": objects,
            "triangles": len(triangles),
            "finite_band": row,
            "extended_band": extended,
            "profile": profile,
        }
        print(f"OBSERVED {name}/{state}: finite radius={row['radius_to_surface_m']:.9f} m", flush=True)
    return {"profile_height_range_above_seat_m": [0.0, end], "states": result}


def _write_review(output, report):
    lines = [
        "# フィンガ保持中の工具アクセス — 静的メッシュ観測 v01",
        "",
        "端子側縁を持つ2本指の既存形状を固定し、締付軸からハンド表面までの水平距離を高さ別に測りました。",
        "上押さえは追加していません。アーム動作や製品動画の再開、工具・指先の採用判定ではありません。",
        "",
        "## 測り方",
        "",
        "対象はEDGE/GUIDEの指先と元の2F-85本体・連動リンクです。各三角形をZ区間で切り取り、",
        "XYへ投影した表面までの最短半径と、元の三角形上の対応点を保存しています。",
        "プロフィールは高さ2 mmごとの区間最小値で、中央高さ1点の値ではありません。",
        "空の区間はnullです。元の世界行列・頂点・面は変更しません。",
        "",
        "端子穴中心の工具軸(X=0,Y=0)と接続面Z=-14 mmは既存比較の値です。",
        "実端子・ソケット・締付ユニットの型式／製作寸法は未選定です。円筒は径比較用で、",
        "ソケットの中空部・ボルトとの接触・カメラ・ケース・もう一方の手／工具を含みません。",
        "",
        "## 従来の工具帯：接続面から3.8〜53.8 mm",
        "",
        "| 候補 | 保存姿勢 | 軸から表面 [mm] | φ12比較円筒との差 [mm] | 最も近いオブジェクト |",
        "|---|---|---:|---:|---|",
    ]
    for name, states in report["observation"]["states"].items():
        for state, data in states.items():
            row = data["finite_band"]
            radius = row["radius_to_surface_m"] * 1000
            lines.append(f"| {name} | {state} | {radius:.6f} | {radius - 6:.6f} | `{row['object']}` |")
    lines += [
        "",
        "## 工具の上部まで比較する理由",
        "",
        "短い工具帯と、4保存姿勢すべての上端まで延ばした帯を別々に記録しています。",
        "延長帯は接続面から3.8〜188.877 mmです。個々の姿勢より高い空の部分も含みます。",
        "短い先端だけの数値を、ソケット・軸・ユニット本体の全長へ適用できません。",
        "差が負なら、その比較円筒の半径内に対象メッシュ表面があることを示します。",
        "正負は実工具の干渉判定・把持力・締結品質・実機成立の受入結果ではありません。",
        "",
        "near/early/openでは高さ120〜122 mm区間に工具軸上の本体表面があり、",
        "その対応点は接続面から120.576976 mmです。clearの対応点は145.576976 mmです。",
        "EDGE/GUIDEとも延長帯の最短半径は0 mでした。両案の指先変更部分が異なっても、",
        "この配置では共通の本体形状が工具軸上に残ります。実工具の外形が未定のため、",
        "指先の最終寸法とハンド本体・締付ユニットの相対配置はまだ確定しません。",
        "",
        "## 参照CADの範囲",
        "",
        "保存Klauke_6R6サンプルは圧着前です。Ampere実装端子との同一性は不明です。",
        "保存CADの全長47 mmと公式表のl=37 mmには寸法区間の対応が未解決です。",
        "この比較を実部品の製作寸法へ転記しません。仮線径14 mmもAmpereの選定値ではありません。",
        "",
        "## 再生成と記録",
        "",
        "`scripts/probe_hand_tool_access_v01.py --source_directory inputs --config data/hand_tool_access_v01.json`",
        "`--output_directory <新規フォルダー>` をNumPyのあるPythonで実行します。",
        "出力済みフォルダーへは上書きしません。入力SHA、解析用既知形状5件、各区間の距離と対応点は",
        "`hand_tool_access_observations_v01.json`を参照。3D比較ページは同梱データだけで開きます。",
        "正式な物理妥当性判定はVaultProtocol V12の独立レビュー経路に留めます。",
        "",
    ]
    (output / "工具アクセス確認記録_v01.md").write_text("\n".join(lines))


def build(source: Path, config_path: Path, output: Path) -> dict:
    """Measure unchanged local meshes in metres and export a self-contained review."""
    settings = json.loads(config_path.read_text())
    paths = [source / settings[key] for key in ("source_mesh_file", "source_config_file", "source_reference_file")]
    before = {path.name: _digest(path) for path in paths}
    assert before[paths[0].name] == settings["source_mesh_sha256"], "Source mesh SHA mismatch"
    payload = json.loads(paths[0].read_text())
    assert payload["config"] == json.loads(paths[1].read_text()), "Embedded/source configuration differs"
    assert settings["axis_xy_m"] == [0.0, 0.0], "Viewer and source guide use the hole-centred axis"
    original_payload = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    checks = _analytic_checks()
    output.mkdir(parents=True, exist_ok=False)
    observed = _observe(payload, settings)
    assert original_payload == json.dumps(payload, sort_keys=True, separators=(",", ":")), "Input payload was mutated"
    assert before == {path.name: _digest(path) for path in paths}, "Input file changed"
    report = {
        "recorded_at": datetime.now().astimezone().isoformat(),
        "scope": settings["scope"],
        "source_sha256": before,
        "config_sha256": _digest(config_path),
        "inputs_unchanged": True,
        "geometry_and_saved_matrices_unchanged": True,
        "analytic_checks": checks,
        "settings": settings,
        "observation": observed,
        "method": "Minimum XY radius of triangle surfaces after exact linear Z-slab clipping, using float64",
        "source_face_index_basis": "Combined triangle list in recorded objects order; no mesh decimation",
        "numerical_check_tolerance_m": 1e-12,
        "solid_occupancy_test": False,
        "dynamic_sweep_test": False,
        "physical_acceptance_verdict": None,
    }
    (output / "hand_tool_access_observations_v01.json").write_text(json.dumps(report, indent=2) + "\n")
    template = Path(__file__).with_name("hand_tool_access_viewer_v01.html")
    page = template.read_text().replace("__MESH_PAYLOAD__", json.dumps(payload, separators=(",", ":")))
    page = page.replace("__OBSERVATION_PAYLOAD__", json.dumps(report, separators=(",", ":")))
    (output / "保持中の工具アクセス_v01.html").write_text(page)
    _write_review(output, report)
    (output / "inputs").mkdir()
    for path in paths:
        shutil.copy2(path, output / "inputs" / path.name)
    (output / "scripts").mkdir()
    for path in (Path(__file__), template):
        shutil.copy2(path, output / "scripts" / path.name)
    (output / "data").mkdir()
    shutil.copy2(config_path, output / "data" / config_path.name)
    return report


def main() -> None:
    """Parse source and fresh output paths for the static observation."""
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source_directory", "config", "output_directory"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    args = parser.parse_args()
    build(args.source_directory, args.config, args.output_directory)
    print("HAND_TOOL_ACCESS_OBSERVED", flush=True)


if __name__ == "__main__":
    main()
