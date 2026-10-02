#!/usr/bin/env python3
"""Plot multiple models' evaluation CSV files together."""

import argparse
import csv
from pathlib import Path


METRICS = {
    'task_success': ('Task success', 'Success', True),
    'grade_success': ('Scene grade', 'Passed grade', True),
    'bowl_stable': ('Bowl stability', 'Stable', True),
    'policy_completed': ('Policy completion', 'Completed', True),
    'runtime_seconds': ('Policy runtime', 'Seconds', False),
    'targets_placed': ('Target cubes placed', 'Cubes', False),
    'distractors_in_bowl': ('Distractor cubes in tray', 'Cubes', False),
}


def truth(value: str) -> float:
    return float(value.lower() in ('1', 'true', 'yes'))


def read_stats(path: Path) -> list[dict[str, str]]:
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    missing = {'attempt', *METRICS} - set(rows[0] if rows else ())
    if missing:
        raise ValueError(f'{path} is empty or missing columns: {", ".join(sorted(missing))}')
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Create combined graphs from results/<model>/stats.csv files'
    )
    parser.add_argument('models', nargs='+', help='model directory names under results/')
    parser.add_argument(
        '--output', type=Path, default=Path('results/model_comparison')
    )
    args = parser.parse_args()
    if len(set(args.models)) != len(args.models):
        parser.error('model names must be unique')
    if any(
        name in ('.', '..') or not all(char.isalnum() or char in '._-' for char in name)
        for name in args.models
    ):
        parser.error('model names may contain only letters, numbers, ., _, and -')

    root = Path(__file__).resolve().parent.parent
    datasets = {
        model: read_stats(root / 'results' / model / 'stats.csv')
        for model in args.models
    }
    output = args.output if args.output.is_absolute() else root / args.output
    output.mkdir(parents=True, exist_ok=True)

    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    for field, (title, ylabel, binary) in METRICS.items():
        figure, axis = plt.subplots(figsize=(8, 4.5))
        for model, rows in datasets.items():
            selected = sorted(rows, key=lambda row: int(row['attempt']))
            attempts = [int(row['attempt']) for row in selected]
            values = [
                truth(row[field]) if binary else float(row[field])
                for row in selected
            ]
            axis.plot(attempts, values, marker='o', markersize=3, label=model)
        axis.set_title(title)
        axis.set_xlabel('Attempt')
        axis.set_ylabel(ylabel)
        axis.grid(alpha=0.25)
        if binary:
            axis.set_ylim(-0.05, 1.05)
            axis.set_yticks((0, 1), ('No', 'Yes'))
        axis.legend()
        figure.tight_layout()
        figure.savefig(output / f'{field}.png', dpi=140)
        plt.close(figure)

    print(output)


if __name__ == '__main__':
    main()
