# xela_taxel_viz_ahv4 Design

## Goals
- Visualize `/x_taxel_ah` tactile data as a 31x26 grid pattern with 368 active cells.
- Support URDF marker mode for Allegro Hand (left/right).
- Reuse the visual style from `xela_taxel_viz_2f` (circle size/color + arrow direction/length).
- Support real-time updates with best-effort performance.

## Inputs
- Topic: `/x_taxel_ah`
- Message: `xela_taxel_msgs/XTaxelSensorTArray`
- Fields used:
- `x_modules[].frame_ids[]` (always `_joint` suffix)
- `x_modules[].forces[]` (optional)
- `x_modules[].taxels[]` (primary data)
- `x_modules[].temps[]` (ignored)

## Data Mapping
### A) Flat index mapping
- Use `taxel_joint_map_new.yaml` to map `frame_id -> flat_index (0..367)`.
- Build a flat array of length 368 per incoming message.
- Matching rule:
  - If `frame_id` exists in the map, place the sample at `flat_index`.
  - If missing, warn and skip.

### B) Pattern grid mapping
- Pattern grid size: 31 rows x 26 columns.
- Traversal order: top-left, row-major (top->bottom, left->right).
- Pattern YAML contains an `index_map` matrix:
  - `-1` for inactive cells
  - `0..367` for active cells (flat indices)
- Visualization uses `index_map` to place each index on the 2D grid.

## Value Selection
- If `forces` is present and `use_forces_if_present=true`, use `forces`.
- Otherwise, use `taxels`.
- Baseline is computed per index; visualization uses `current - baseline`.

## Normalization
- Use the same normalization model as `xela_taxel_viz_2f`.
- Ranges (from tech spec):
  - X/Y: baseline +/- 1350
  - Z: baseline to baseline + 16000
- Z negative deltas are clamped to 0 for normalization.

## Visualization Style
- Same as `xela_taxel_viz_2f`:
  - Circle radius and color scale with normalized magnitude.
  - Arrow length scales with the same magnitude.
  - Arrow direction uses XY direction (default).

## Parameters (draft)
- `in_topic`, `out_topic`, `frame_id` (default `x_taxel_ah_viz` for grid mode)
- `pattern_yaml` (pattern file path)
- `mapping_yaml` (taxel_joint_map_new.yaml path)
- `hand_side` (left/right prefix rewrite)
- `grid_rows`, `grid_cols`, `cell_size`, `origin_x`, `origin_y`
- `use_forces_if_present`
- `baseline_duration_sec`
- `use_axis_normalization`, `xy_range`, `z_range`
- `circle_*`, `arrow_*`, `grid_*`, `color_*`
- `marker_stamp_mode`, `marker_time_offset_sec`

## Tooling: CSV -> Pattern YAML
### Recommended workflow
1) Export the Excel sheet to CSV (31 rows x 26 cols, empty cells allowed).
2) Convert CSV to YAML using a small Python script.

### Example script (Python)
```
python3 - <<'PY'
import csv
import yaml
from pathlib import Path

csv_path = Path('pattern.csv')
out_path = Path('pattern.yaml')

rows = []
with csv_path.open(newline='') as f:
    reader = csv.reader(f)
    for row in reader:
        parsed = []
        for cell in row:
            cell = cell.strip()
            if cell == '' or cell.lower() == 'nan':
                parsed.append(-1)
            else:
                parsed.append(int(float(cell)))
        rows.append(parsed)

payload = {
    'pattern': {
        'rows': len(rows),
        'cols': len(rows[0]) if rows else 0,
        'index_map': rows,
    }
}

with out_path.open('w') as f:
    yaml.safe_dump(payload, f, sort_keys=False)

print('wrote', out_path)
PY
```

## Open Questions
- Confirm whether forces should override taxels (default: yes).
- Confirm QoS policy (best-effort suggested).
- Confirm default marker timestamp offset per platform.

## Risks
- Missing `frame_id` mapping will leave holes in the grid.
- Performance may degrade if update rate is very high; consider throttling if needed.
