#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MORAI 2D 단순화 링크를 '넓은 도로 면(Polygon)'으로 병합:
- 각 차선 중심선(LineString)을 (평균폭/2 + margin) 으로 버퍼링
- 모든 버퍼를 unary_union으로 디졸브 → 연속된 도로 면 Polygon 획득
- 결과를 GeoJSON FeatureCollection(Polygon/MultiPolygon)으로 저장
"""
import json, argparse, os
from statistics import mean
from typing import Any, Dict, List
from shapely.geometry import LineString, mapping
from shapely.ops import unary_union

def parse_args():
    p = argparse.ArgumentParser(description="Merge lanes to road surfaces (Polygon) via buffer+dissolve.")
    p.add_argument("-i","--input", default="link_set_2d_simplified.json",
                   help="Input lane-level JSON (array of links)")
    p.add_argument("-o","--output", default="merged_road_surfaces.geojson",
                   help="Output GeoJSON file path")
    p.add_argument("--width-default", type=float, default=4,
                   help="Default lane width when missing")
    p.add_argument("--margin", type=float, default=0.45,
                   help="Extra meters added to half-width (helps dissolve gaps)")
    p.add_argument("--simplify", type=float, default=0.00,
                   help="Optional geometry simplification tolerance (0=off)")
    return p.parse_args()

def avg_lane_width_m(l: Dict[str,Any], default: float) -> float:
    try:
        ws = float(l.get("width_start", default) or default)
        we = float(l.get("width_end", default) or default)
    except (TypeError, ValueError):
        ws = we = default
    return (ws + we) / 2.0

def to_line(points: List[List[float]]) -> LineString:
    # 좌표 정제
    coords = []
    for c in points or []:
        if isinstance(c, (list,tuple)) and len(c)>=2:
            x,y = c[0], c[1]
            if isinstance(x,(int,float)) and isinstance(y,(int,float)):
                coords.append((float(x), float(y)))
    return LineString(coords) if len(coords)>=2 else None

def main():
    args = parse_args()
    with open(args.input, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, list):
        raise ValueError("Input must be a JSON array of lane/link objects.")

    buffers = []
    dropped = 0
    for l in data:
        ln = to_line(l.get("points"))
        if ln is None:
            dropped += 1
            continue
        w = avg_lane_width_m(l, args.width_default)
        half = max(0.1, w * 0.5) + args.margin
        # end_cap_style=1(round), join_style=1(round) → 곡선/교차부 이음새가 자연스러움
        buf = ln.buffer(half + 3, cap_style=1, join_style=1)
        buffers.append(buf)

    if not buffers:
        raise RuntimeError("No valid lane geometries found.")

    merged = unary_union(buffers)                 # dissolve
    if args.simplify > 0:
        merged = merged.simplify(args.simplify)   # 선택: 단순화

    # GeoJSON 내보내기
    def geom_to_features(geom, props):
        # MultiPolygon/Polygon 모두 처리
        if geom.geom_type == "Polygon":
            return [{"type":"Feature","properties":props,"geometry":mapping(geom)}]
        elif geom.geom_type == "MultiPolygon":
            return [{"type":"Feature","properties":props,"geometry":mapping(g)} for g in geom.geoms]
        else:
            return []

    features = geom_to_features(merged, {
        "source": os.path.basename(args.input),
        "method": "buffer+dissolve",
        "width_default_m": args.width_default,
        "margin_m": args.margin,
        "simplify": args.simplify
    })

    out = {"type":"FeatureCollection","name":"merged_road_surfaces","features":features}
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False)
    print(f"lanes: {len(data)}, dropped_no_geom: {dropped}, surfaces: {len(features)}")
    print(f"Wrote: {os.path.abspath(args.output)}")

if __name__ == "__main__":
    main()