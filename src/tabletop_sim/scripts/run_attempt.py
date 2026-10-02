#!/usr/bin/env python3
"""Run one immutable policy attempt and collect public and private results."""

import argparse
import csv
import importlib.util
import json
from pathlib import Path
import shutil
import signal
import time

from grading import grade, public_grade, read_poses
from image_file import save_png
from manipulation_api import Robot
from reset_environment import reset_and_verify


STAT_FIELDS = (
    'attempt',
    'seed',
    'task_success',
    'grade_success',
    'bowl_stable',
    'policy_completed',
    'runtime_seconds',
    'targets_placed',
    'targets_required',
    'distractors_in_bowl',
    'error',
    'run_directory',
)


def save_row(path: Path, row: dict[str, object]) -> None:
    rows: list[dict[str, str]] = []
    if path.exists():
        with path.open(newline='', encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
    key = str(row['attempt'])
    if any(item['attempt'] == key for item in rows):
        raise ValueError(f'statistics already contain attempt {key}')
    rows.append({field: str(row.get(field, '')) for field in STAT_FIELDS})
    rows.sort(key=lambda item: int(item['attempt']))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=STAT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def row_exists(path: Path, attempt: int) -> bool:
    return any(int(row['attempt']) == attempt for row in rows(path))


def load_policy(path: Path):
    spec = importlib.util.spec_from_file_location('attempt_policy', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load policy from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    policy = getattr(module, 'policy', None)
    if not callable(policy):
        raise RuntimeError(f'{path} does not define callable policy(robot)')
    return policy


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n', encoding='utf-8')


class RecordingRobot:
    """Expose the policy API while recording a frame after every action."""

    def __init__(self, robot: Robot, output: Path, initial_frame) -> None:
        self._robot = robot
        self._frames = output / 'frames'
        self._actions = output / 'actions.json'
        self._events: list[dict[str, object]] = []
        self._started = time.monotonic()
        self._record('initial', {}, initial_frame)

    def _record(
        self,
        action: str,
        arguments: dict[str, object],
        frame=None,
        command_error: str | None = None,
    ) -> None:
        index = len(self._events)
        event: dict[str, object] = {
            'index': index,
            'action': action,
            'arguments': arguments,
            'elapsed_seconds': round(time.monotonic() - self._started, 3),
        }
        if command_error is not None:
            event['command_error'] = command_error
        try:
            captured = frame or self._robot.capture_camera()
            filename = f'{index:03d}_{action}.png'
            save_png(captured, self._frames / filename)
            event['image'] = f'frames/{filename}'
            event['camera_timestamp_ns'] = captured.timestamp_ns
        except TimeoutError:
            raise
        except Exception as error:
            event['capture_error'] = f'{type(error).__name__}: {error}'
        self._events.append(event)
        write_json(self._actions, {'events': self._events})

    def _run_action(self, action: str, arguments: dict[str, object], command):
        try:
            result = command()
        except Exception as error:
            self._record(action, arguments, command_error=f'{type(error).__name__}: {error}')
            raise
        self._record(action, arguments)
        return result

    def capture_camera(self, timeout: float = 5.0):
        return self._robot.capture_camera(timeout)

    def get_end_effector_pose(self, timeout: float = 5.0):
        return self._robot.get_end_effector_pose(timeout)

    def move_to(self, position, orientation=None):
        target = tuple(float(value) for value in position)
        target_orientation = (
            None if orientation is None else tuple(float(value) for value in orientation)
        )
        return self._run_action(
            'move_to',
            {
                'position': list(target),
                'orientation': None if target_orientation is None else list(target_orientation),
            },
            lambda: self._robot.move_to(target, target_orientation),
        )

    def move_relative(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0):
        offsets = {'dx': float(dx), 'dy': float(dy), 'dz': float(dz)}
        return self._run_action(
            'move_relative',
            offsets,
            lambda: self._robot.move_relative(**offsets),
        )

    def open_gripper(self) -> None:
        self._run_action('open_gripper', {}, self._robot.open_gripper)

    def close_gripper(self) -> None:
        self._run_action('close_gripper', {}, self._robot.close_gripper)

    def record_final(self, frame) -> None:
        self._record('final', {}, frame)


def main() -> None:
    parser = argparse.ArgumentParser(description='Run one policy and collect feedback')
    parser.add_argument('--attempt', type=int, required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--policy-file', type=Path, default=Path('results/policy.py'))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--private-root', type=Path, default=Path('private'))
    parser.add_argument('--stats', type=Path)
    parser.add_argument('--max-runtime', type=int, default=120)
    parser.add_argument('--reset-timeout', type=float, default=30.0)
    parser.add_argument('--settle-seconds', type=float, default=1.0)
    args = parser.parse_args()
    if args.attempt <= 0 or args.max_runtime <= 0 or args.reset_timeout <= 0.0:
        parser.error('attempt, runtime, and reset timeout must be positive')
    if not 0 <= args.seed <= 0xFFFFFFFF or args.settle_seconds < 0.0:
        parser.error('seed must be uint32 and settle time cannot be negative')

    policy_path = args.policy_file.resolve()
    workspace = policy_path.parent
    output = (args.output or workspace / f'attempt{args.attempt:03d}').resolve()
    stats_path = (args.stats or workspace / 'stats.csv').resolve()
    private_root = args.private_root.resolve()
    private_output = private_root / f'attempt{args.attempt:03d}'

    if private_root == workspace or workspace in private_root.parents:
        parser.error('--private-root must be outside the model workspace')
    if output.exists() or private_output.exists() or row_exists(stats_path, args.attempt):
        parser.error(f'attempt {args.attempt} already exists')

    if args.attempt > 1:
        previous_public = (
            workspace / f'attempt{args.attempt - 1:03d}' / 'public_grade.json'
        )
        if not previous_public.is_file() or not row_exists(stats_path, args.attempt - 1):
            parser.error(f'attempt {args.attempt} requires completed attempt {args.attempt - 1}')
        previous_result = json.loads(previous_public.read_text(encoding='utf-8'))
        if previous_result.get('seed') != args.seed:
            parser.error('all attempts must use the same seed')

    output.mkdir(parents=True)
    private_output.mkdir(parents=True)
    stored_policy = output / 'policy.py'
    if policy_path.is_file():
        shutil.copy2(policy_path, stored_policy)

    def finish_failure(status: str, error: Exception, runtime: float = 0.0) -> None:
        message = f'{type(error).__name__}: {error}'
        execution = {
            'attempt': args.attempt,
            'seed': args.seed,
            'policy_status': status,
            'runtime_seconds': round(runtime, 3),
            'task_success': False,
            'grade_available': False,
            'error': message,
        }
        public = {
            **execution,
            'criteria': {
                'bowl_stable': False,
                'repeated_color_cubes_in_bowl': {'placed': 0, 'required': 2},
                'unique_color_cubes_in_bowl': 0,
            },
        }
        write_json(output / 'public_grade.json', public)
        write_json(private_output / 'private_grade.json', execution)
        save_row(
            stats_path,
            {
                'attempt': args.attempt,
                'seed': args.seed,
                'task_success': False,
                'grade_success': False,
                'bowl_stable': False,
                'policy_completed': False,
                'runtime_seconds': round(runtime, 3),
                'targets_placed': 0,
                'targets_required': 2,
                'distractors_in_bowl': 0,
                'error': message,
                'run_directory': output,
            },
        )
        print(json.dumps(public, indent=2, sort_keys=True))
        raise SystemExit(1)

    try:
        policy = load_policy(stored_policy)
    except Exception as error:
        finish_failure('load_failed', error)

    try:
        reset_and_verify(args.seed, args.reset_timeout, args.settle_seconds)
    except Exception as error:
        finish_failure('reset_failed', error)

    status = 'completed'
    error_message = None
    runtime = 0.0

    def timeout_handler(_signum, _frame) -> None:
        raise TimeoutError(f'policy exceeded {args.max_runtime} seconds')

    try:
        with Robot() as robot:
            initial_frame = robot.capture_camera()
            save_png(initial_frame, output / 'initial.png')
            recording_robot = RecordingRobot(robot, output, initial_frame)
            started = time.monotonic()
            previous_handler = signal.signal(signal.SIGALRM, timeout_handler)
            signal.alarm(args.max_runtime)
            try:
                policy(recording_robot)
            except Exception as error:
                status = 'failed'
                error_message = f'{type(error).__name__}: {error}'
            finally:
                signal.alarm(0)
                signal.signal(signal.SIGALRM, previous_handler)
                runtime = time.monotonic() - started
            time.sleep(args.settle_seconds)
            final_frame = robot.capture_camera()
            save_png(final_frame, output / 'final.png')
            recording_robot.record_final(final_frame)
    except Exception as error:
        status = 'failed'
        error_message = f'{type(error).__name__}: {error}'

    try:
        private = grade(read_poses('/world/tabletop/pose/info', 5.0))
    except Exception as error:
        finish_failure('grade_failed', error, runtime)

    public = public_grade(private)
    execution = {
        'attempt': args.attempt,
        'seed': args.seed,
        'policy_status': status,
        'runtime_seconds': round(runtime, 3),
    }
    if error_message is not None:
        execution['error'] = error_message
    effective_success = status == 'completed' and bool(private['success'])
    public = {**execution, **public, 'task_success': effective_success}
    private = {**execution, **private}

    write_json(output / 'public_grade.json', public)
    write_json(private_output / 'private_grade.json', private)
    criteria = public['criteria']
    save_row(
        stats_path,
        {
            'attempt': args.attempt,
            'seed': args.seed,
            'task_success': effective_success,
            'grade_success': private['success'],
            'bowl_stable': criteria['bowl_stable'],
            'policy_completed': status == 'completed',
            'runtime_seconds': round(runtime, 3),
            'targets_placed': criteria['repeated_color_cubes_in_bowl']['placed'],
            'targets_required': criteria['repeated_color_cubes_in_bowl']['required'],
            'distractors_in_bowl': criteria['unique_color_cubes_in_bowl'],
            'error': error_message or '',
            'run_directory': output,
        },
    )
    print(json.dumps(public, indent=2, sort_keys=True))
    raise SystemExit(0 if effective_success else 1)


if __name__ == '__main__':
    main()
