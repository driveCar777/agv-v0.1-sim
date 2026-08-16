#!/usr/bin/env python3
"""把办公室 .smap 导出为 Nav2 可用的 map.yaml + map.pgm。

用法:
  python scripts/export_smap_to_navmap.py
  python scripts/export_smap_to_navmap.py --smap path/to/x.smap --out maps/office
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.smap_loader import default_smap_candidates, load_smap  # noqa: E402


def export(smap_path: Path, out_dir: Path, res: float = 0.05) -> Path:
    loaded = load_smap(smap_path, plan_res=res, inflate_m=0.0, cloud_max=200000)
    out_dir.mkdir(parents=True, exist_ok=True)
    width = int(math.ceil((loaded.max_x - loaded.min_x) / res)) + 1
    height = int(math.ceil((loaded.max_y - loaded.min_y) / res)) + 1
    # PGM: 0 occupied, 254 free, 205 unknown — 这里未扫到视为 free（办公室已扫图）
    grid = bytearray([254] * (width * height))

    for x, y in loaded.cloud:
        ix = int(math.floor((x - loaded.min_x) / res))
        iy = int(math.floor((y - loaded.min_y) / res))
        if 0 <= ix < width and 0 <= iy < height:
            # pgm 行从顶部开始 → 翻转 y
            row = (height - 1 - iy) * width + ix
            grid[row] = 0

    pgm = out_dir / "map.pgm"
    with pgm.open("wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        f.write(grid)

    yaml_path = out_dir / "map.yaml"
    yaml_path.write_text(
        "\n".join(
            [
                f"image: map.pgm",
                f"resolution: {res}",
                f"origin: [{loaded.min_x:.4f}, {loaded.min_y:.4f}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                f"# source: {smap_path}",
                f"# name: {loaded.name}",
                "",
            ]
        ),
        encoding="utf-8",
    )

    # POI 旁路文件，便于导航组件/任务
    pois = out_dir / "pois.yaml"
    lines = ["pois:"]
    for p in loaded.pois:
        lines.append(f"  - id: {p.id}")
        lines.append(f"    x: {p.x}")
        lines.append(f"    y: {p.y}")
        lines.append(f"    kind: {p.kind}")
    pois.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return yaml_path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--smap", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=ROOT / "maps" / "office_nav")
    ap.add_argument("--res", type=float, default=0.05)
    args = ap.parse_args()
    smap = args.smap
    if smap is None:
        cands = default_smap_candidates()
        if not cands:
            print("no smap found")
            return 1
        smap = max(cands, key=lambda p: p.stat().st_size)
    yaml_path = export(smap, args.out, res=args.res)
    print(f"exported {smap} -> {yaml_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
