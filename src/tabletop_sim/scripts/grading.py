"""Privileged pose grading shared by the grader and attempt runner."""

from queue import Empty, Queue
import math
import struct
import time
from typing import Iterator

from gz.transport13 import Node, SubscribeOptions


TARGETS = ('red_cube', 'red_cube_stacked_lower')
DISTRACTORS = ('green_cube_stacked_upper', 'yellow_cube')
OBJECTS = TARGETS + DISTRACTORS
TRAY = 'blue_tray'
RESET_MODELS = OBJECTS + (TRAY,)
INITIAL_POSES = {
    'red_cube': (0.46, 0.18, 0.025, 0.0, 0.0, 0.0, 1.0),
    'red_cube_stacked_lower': (0.68, 0.10, 0.025, 0.0, 0.0, 0.0, 1.0),
    'green_cube_stacked_upper': (0.68, 0.10, 0.075, 0.0, 0.0, 0.0, 1.0),
    'yellow_cube': (0.63, -0.02, 0.025, 0.0, 0.0, 0.0, 1.0),
    TRAY: (0.52, -0.22, 0.012, 0.0, 0.0, 0.0, 1.0),
}
TRAY_POSITION_TOLERANCE = 0.02
TRAY_ORIENTATION_TOLERANCE_DEGREES = 10.0
POSE_VECTOR_TYPE = 'gz.msgs.Pose_V'


def _read_varint(data: bytes, offset: int) -> tuple[int, int]:
    value = 0
    shift = 0
    while offset < len(data) and shift < 70:
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, offset
        shift += 7
    raise RuntimeError('invalid Gazebo protobuf varint')


def _protobuf_fields(data: bytes) -> Iterator[tuple[int, int, int | bytes]]:
    offset = 0
    while offset < len(data):
        key, offset = _read_varint(data, offset)
        number, wire_type = key >> 3, key & 0x07
        if wire_type == 0:
            value, offset = _read_varint(data, offset)
        elif wire_type == 1:
            value = data[offset : offset + 8]
            offset += 8
        elif wire_type == 2:
            size, offset = _read_varint(data, offset)
            value = data[offset : offset + size]
            offset += size
        elif wire_type == 5:
            value = data[offset : offset + 4]
            offset += 4
        else:
            raise RuntimeError(f'unsupported Gazebo protobuf wire type {wire_type}')
        if offset > len(data):
            raise RuntimeError('truncated Gazebo protobuf message')
        yield number, wire_type, value


def _decode_vector(data: bytes, fields: tuple[int, ...]) -> tuple[float, ...]:
    values: dict[int, float] = {}
    for number, wire_type, value in _protobuf_fields(data):
        if number in fields and wire_type == 1 and isinstance(value, bytes):
            values[number] = struct.unpack('<d', value)[0]
    return tuple(values.get(number, 0.0) for number in fields)


def _decode_pose_vector(packet: bytes) -> dict[str, tuple[float, ...]]:
    poses: dict[str, tuple[float, ...]] = {}
    for number, wire_type, pose_data in _protobuf_fields(packet):
        if number != 2 or wire_type != 2 or not isinstance(pose_data, bytes):
            continue
        name = None
        position_data = None
        orientation_data = None
        for pose_number, pose_wire, value in _protobuf_fields(pose_data):
            if pose_number == 2 and pose_wire == 2 and isinstance(value, bytes):
                name = value.decode('utf-8')
            elif pose_number == 4 and pose_wire == 2 and isinstance(value, bytes):
                position_data = value
            elif pose_number == 5 and pose_wire == 2 and isinstance(value, bytes):
                orientation_data = value
        if name is None or position_data is None or orientation_data is None:
            continue
        position = _decode_vector(position_data, (2, 3, 4))
        orientation = _decode_vector(orientation_data, (2, 3, 4, 5))
        poses[name] = position + orientation
    return poses


def _normalize_quaternion(values: tuple[float, ...]) -> tuple[float, float, float, float]:
    magnitude = math.sqrt(sum(value * value for value in values))
    if magnitude < 1e-9:
        raise ValueError('pose contains a zero quaternion')
    return tuple(value / magnitude for value in values)  # type: ignore[return-value]


def _multiply_quaternions(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _to_tray_frame(
    position: tuple[float, ...], tray_pose: tuple[float, ...]
) -> tuple[float, float, float]:
    dx, dy, dz = (position[index] - tray_pose[index] for index in range(3))
    qx, qy, qz, qw = _normalize_quaternion(tray_pose[3:7])
    inverse = (-qx, -qy, -qz, qw)
    local = _multiply_quaternions(
        _multiply_quaternions(inverse, (dx, dy, dz, 0.0)),
        (qx, qy, qz, qw),
    )
    return local[:3]


def tray_status(tray_pose: tuple[float, ...]) -> dict[str, object]:
    initial = INITIAL_POSES[TRAY]
    position_error = math.dist(tray_pose[:3], initial[:3])
    actual_q = _normalize_quaternion(tray_pose[3:7])
    initial_q = _normalize_quaternion(initial[3:7])
    dot = min(1.0, abs(sum(a * b for a, b in zip(actual_q, initial_q))))
    orientation_error = math.degrees(2.0 * math.acos(dot))
    stable = (
        position_error <= TRAY_POSITION_TOLERANCE
        and orientation_error <= TRAY_ORIENTATION_TOLERANCE_DEGREES
    )
    return {
        'stable': stable,
        'position': [round(value, 4) for value in tray_pose[:3]],
        'orientation': [round(value, 5) for value in actual_q],
        'position_error_m': round(position_error, 4),
        'orientation_error_degrees': round(orientation_error, 2),
    }


def inside_bowl(position: tuple[float, ...], tray_pose: tuple[float, ...]) -> bool:
    x, y, z = _to_tray_frame(position, tray_pose)
    return abs(x) <= 0.075 and abs(y) <= 0.055 and 0.018 <= z <= 0.088


def grade(poses: dict[str, tuple[float, ...]]) -> dict[str, object]:
    tray = tray_status(poses[TRAY])
    objects = {
        name: {
            'role': 'target' if name in TARGETS else 'distractor',
            'position': [round(value, 4) for value in poses[name][:3]],
            'inside_bowl': inside_bowl(poses[name], poses[TRAY]),
        }
        for name in OBJECTS
    }
    success = bool(tray['stable']) and all(
        objects[name]['inside_bowl'] for name in TARGETS
    ) and all(not objects[name]['inside_bowl'] for name in DISTRACTORS)
    return {'success': success, 'tray': tray, 'objects': objects}


def public_grade(private: dict[str, object]) -> dict[str, object]:
    objects = private['objects']
    tray = private['tray']
    return {
        'task_success': private['success'],
        'criteria': {
            'bowl_stable': tray['stable'],
            'repeated_color_cubes_in_bowl': {
                'placed': sum(bool(objects[name]['inside_bowl']) for name in TARGETS),
                'required': len(TARGETS),
            },
            'unique_color_cubes_in_bowl': sum(
                bool(objects[name]['inside_bowl']) for name in DISTRACTORS
            ),
        },
    }


def read_poses(topic: str, timeout: float) -> dict[str, tuple[float, ...]]:
    poses: dict[str, tuple[float, ...]] = {}
    packets: Queue[bytes] = Queue()

    def on_raw(message: bytes, _message_info: object) -> None:
        packets.put(bytes(message))

    node = Node()
    if not node.subscribe_raw(
        topic, on_raw, POSE_VECTOR_TYPE, SubscribeOptions()
    ):
        raise RuntimeError(f'could not subscribe to {topic}')
    try:
        deadline = time.monotonic() + timeout
        while not all(name in poses for name in RESET_MODELS):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            try:
                packet = packets.get(timeout=remaining)
            except Empty:
                break
            poses.update(
                (name, pose)
                for name, pose in _decode_pose_vector(packet).items()
                if name in RESET_MODELS
            )
        missing = [name for name in RESET_MODELS if name not in poses]
        if missing:
            raise RuntimeError(f'timed out waiting for poses: {", ".join(missing)}')
        return poses
    finally:
        node.unsubscribe(topic)
