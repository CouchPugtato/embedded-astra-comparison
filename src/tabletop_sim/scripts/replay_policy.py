#!/usr/bin/env python3
"""Reset the scene and execute one saved policy without recording a new attempt."""

import argparse
import importlib.util
from pathlib import Path
import signal

from manipulation_api import Robot
from reset_environment import reset_and_verify


def load_policy(path: Path):
    spec = importlib.util.spec_from_file_location('replay_policy_module', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load policy from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    policy = getattr(module, 'policy', None)
    if not callable(policy):
        raise RuntimeError(f'{path} does not define callable policy(robot)')
    return policy


def main() -> None:
    parser = argparse.ArgumentParser(description='Replay one saved policy')
    parser.add_argument('--policy-file', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--max-runtime', type=int, default=120)
    parser.add_argument('--reset-timeout', type=float, default=30.0)
    args = parser.parse_args()
    if args.max_runtime <= 0 or args.reset_timeout <= 0.0:
        parser.error('runtime and reset timeout must be positive')

    policy_path = args.policy_file.resolve()
    policy = load_policy(policy_path)
    reset_and_verify(args.seed, args.reset_timeout, 1.0)

    def timeout_handler(_signum, _frame) -> None:
        raise TimeoutError(f'policy exceeded {args.max_runtime} seconds')

    previous_handler = signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(args.max_runtime)
    try:
        with Robot() as robot:
            policy(robot)
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, previous_handler)
    print(f'Replay complete: {policy_path}')


if __name__ == '__main__':
    main()
