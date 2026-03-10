#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path
import yaml


def load_csv(path: Path):
    rows = []
    with path.open(newline='') as f:
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
    return rows


def main():
    parser = argparse.ArgumentParser(description='Convert CSV pattern to YAML.')
    parser.add_argument('csv', type=Path, help='Input CSV path')
    parser.add_argument('-o', '--out', type=Path, required=True, help='Output YAML path')
    args = parser.parse_args()

    rows = load_csv(args.csv)
    if not rows:
        raise SystemExit('CSV has no rows')
    cols = len(rows[0])
    if any(len(r) != cols for r in rows):
        raise SystemExit('CSV rows have inconsistent column counts')

    payload = {
        'pattern': {
            'rows': len(rows),
            'cols': cols,
            'index_map': rows,
        }
    }

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('w') as f:
        yaml.safe_dump(payload, f, sort_keys=False)

    print(f'Wrote {args.out}')


if __name__ == '__main__':
    main()
