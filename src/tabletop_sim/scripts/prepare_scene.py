#!/usr/bin/env python3
"""Reset, verify, and capture the scene supplied to the model."""

import argparse
import json
from pathlib import Path

from image_file import save_png
from manipulation_api import Robot
from reset_environment import reset_and_verify


def main() -> None:
    parser = argparse.ArgumentParser(description='Prepare and capture the evaluation scene')
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--settle-seconds', type=float, default=1.0)
    parser.add_argument('--output', type=Path, default=Path('prompts/initial.png'))
    args = parser.parse_args()

    result = reset_and_verify(args.seed, args.timeout, args.settle_seconds)
    with Robot() as robot:
        save_png(robot.capture_camera(args.timeout), args.output)
    result['initial_image'] = str(args.output)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
