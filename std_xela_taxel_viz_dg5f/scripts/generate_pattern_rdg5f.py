#!/usr/bin/env python3
"""Regenerates config/patterns/pattern_rdg5f.yaml from the authoritative sources:

  - xela_server2_dg5f/config/r_server_model_joint_map.yaml (flat taxel index -> joint name)
  - config/models/XDG5FR/grid.yaml (module block offsets + layout_templates)

xela_taxel_sidecar_dg5f's C++ bridge (forked from xela_taxel_sidecar_ah) needs a
dense {rows, cols, index_map} pattern file. Rather than hand-deriving that once
and hardcoding the result, this script recomputes it directly from the same two
config files std_xela_taxel_viz_dg5f's RViz node reads at runtime, so the two
renderers stay in sync and re-running this after either file changes (e.g. the
joint map gets regenerated with a different flat-index order, or grid.yaml's
block offsets are retuned) reproduces a correct pattern_rdg5f.yaml.

Usage:
  python3 generate_pattern_rdg5f.py

Taxel placement rule (matches std_xela_taxel_viz_dg5f.cpp's resolveTaxels()):
for a module with a `layout` template, each taxel's cell is looked up by its
own dot number (the trailing _NN in its joint name) in that template's
dot_positions -- NOT by its position/order in r_server_model_joint_map.yaml.
For a module without a `layout` (mid/prox/palm today), taxels are placed
row-major in the order they appear (by flat index) in the joint map, mirroring
the C++ node's fallback behavior.
"""
import re
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
JOINT_MAP_PATH = (
    REPO_ROOT / "xela_apps" / "xela_server2_dg5f" / "config" / "r_server_model_joint_map.yaml"
)
GRID_YAML_PATH = Path(__file__).resolve().parents[1] / "config" / "models" / "XDG5FR" / "grid.yaml"
OUTPUT_PATH = Path(__file__).resolve().parents[1] / "config" / "patterns" / "pattern_rdg5f.yaml"

JOINT_NAME_RE = re.compile(r"^x_taxel_\d+_(.+)_(\d{1,3})_joint$")


def load_joint_map(path):
    data = yaml.safe_load(path.read_text())
    mapping = data["server_model_joint_map"]
    return {int(k): v for k, v in mapping.items()}


def load_grid_config(path):
    docs = list(yaml.safe_load_all(path.read_text()))
    merged = {}
    for doc in docs:
        if doc:
            merged.update(doc)
    return merged


def main():
    joint_map = load_joint_map(JOINT_MAP_PATH)
    grid_cfg = load_grid_config(GRID_YAML_PATH)

    canvas_rows = grid_cfg["/**"]["ros__parameters"]["canvas_rows"]
    canvas_cols = grid_cfg["/**"]["ros__parameters"]["canvas_cols"]
    layout_templates = grid_cfg.get("layout_templates", {})
    modules_cfg = {m["name"]: m for m in grid_cfg["modules"]}

    grid = [[-1] * canvas_cols for _ in range(canvas_rows)]

    # Group joint-map entries by module, preserving flat-index (arrival) order
    # within each module, for the fallback (no-layout) placement rule.
    by_module = {}
    for flat_index in sorted(joint_map.keys()):
        joint_name = joint_map[flat_index]
        m = JOINT_NAME_RE.match(joint_name)
        if not m:
            print(f"WARNING: joint name '{joint_name}' does not match expected pattern; skipping",
                  file=sys.stderr)
            continue
        module_key, dot_str = m.group(1), m.group(2)
        by_module.setdefault(module_key, []).append((flat_index, int(dot_str)))

    placed = 0
    for module_key, entries in by_module.items():
        module_cfg = modules_cfg.get(module_key)
        if module_cfg is None:
            print(f"WARNING: module '{module_key}' not found in grid.yaml modules list; skipping",
                  file=sys.stderr)
            continue
        offset_row = module_cfg["offset_row"]
        offset_col = module_cfg["offset_col"]
        grid_cols = module_cfg["grid_cols"]
        layout_name = module_cfg.get("layout")
        template = layout_templates.get(layout_name) if layout_name else None

        if template is not None:
            dot_positions = {int(k): v for k, v in template["dot_positions"].items()}
            for flat_index, dot in entries:
                rc = dot_positions.get(dot)
                if rc is None:
                    print(f"WARNING: module '{module_key}' dot {dot} not in layout "
                          f"'{layout_name}'; skipping taxel (flat_index={flat_index})",
                          file=sys.stderr)
                    continue
                row, col = offset_row + rc[0], offset_col + rc[1]
                grid[row][col] = flat_index
                placed += 1
        else:
            # Fallback: row-major in arrival (flat-index) order, matching the
            # C++ node's behavior for modules with no confirmed dot layout.
            for local_idx, (flat_index, _dot) in enumerate(entries):
                row = offset_row + local_idx // grid_cols
                col = offset_col + local_idx % grid_cols
                grid[row][col] = flat_index
                placed += 1

    if placed != len(joint_map):
        print(f"WARNING: placed {placed} taxels but joint map has {len(joint_map)}", file=sys.stderr)

    header = (
        "# Auto-generated by generate_pattern_rdg5f.py -- DO NOT hand-edit.\n"
        "# Regenerate by re-running that script after changing either\n"
        "# xela_server2_dg5f/config/r_server_model_joint_map.yaml or\n"
        "# config/models/XDG5FR/grid.yaml. Each cell holds the flat taxel index\n"
        "# (0-123) or -1 for empty; derived from grid.yaml's layout_templates\n"
        "# (dot-number keyed, robust to joint-map reordering) for modules that\n"
        "# have one, and from vendor arrival order for the rest (see grid.yaml's\n"
        "# own header comment on that caveat).\n"
    )
    out = {"pattern": {"rows": canvas_rows, "cols": canvas_cols, "index_map": grid}}
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(OUTPUT_PATH, "w") as f:
        f.write(header)
        yaml.dump(out, f, default_flow_style=None, sort_keys=False)
    print(f"Wrote {OUTPUT_PATH} ({placed} taxels placed)")


if __name__ == "__main__":
    main()
