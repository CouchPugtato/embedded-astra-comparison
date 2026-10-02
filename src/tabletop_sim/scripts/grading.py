"""Privileged pose grading shared by the grader and attempt runner."""

from queue import Empty, Queue
import struct
import time
from typing import Iterator

from gz.transport13 import Node, SubscribeOptions


TARGETS = ('red_cube', 'red_cube_stacked_lower')
DISTRACTORS = ('green_cube_stacked_upper', 'yellow_cube')
OBJECTS = TARGETS + DISTRACTORS
INITIAL_POSITIONS = {
    'red_cube': (0.46, 0.18, 0.025),
    'red_cube_stacked_lower': (0.68, 0.10, 0.025),
    'green_cube_stacked_upper': (0.68, 0.10, 0.075),
    'yellow_cube': (0.63, -0.02, 0.025),
}
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


def _decode_pose_vector(packet: bytes) -> dict[str, tuple[float, float, float]]:
    positions: dict[str, tuple[float, float, float]] = {}
    for number, wire_type, pose_data in _protobuf_fields(packet):
        if number != 2 or wire_type != 2 or not isinstance(pose_data, bytes):
            continue
        name = None
        vector_data = None
        for pose_number, pose_wire, value in _protobuf_fields(pose_data):
            if pose_number == 2 and pose_wire == 2 and isinstance(value, bytes):
                name = value.decode('utf-8')
            elif pose_number == 4 and pose_wire == 2 and isinstance(value, bytes):
                vector_data = value
        if name is None or vector_data is None:
            continue
        coordinates: dict[int, float] = {}
        for vector_number, vector_wire, value in _protobuf_fields(vector_data):
            if vector_number in (2, 3, 4) and vector_wire == 1 and isinstance(value, bytes):
                coordinates[vector_number] = struct.unpack('<d', value)[0]
        if all(number in coordinates for number in (2, 3, 4)):
            positions[name] = (
                coordinates[2],
                coordinates[3],
                coordinates[4],
            )
    return positions


def inside_bowl(position: tuple[float, float, float]) -> bool:
    x, y, z = position
    return abs(x - 0.52) <= 0.075 and abs(y + 0.22) <= 0.055 and 0.03 <= z <= 0.10


def grade(positions: dict[str, tuple[float, float, float]]) -> dict[str, object]:
    objects = {
        name: {
            'role': 'target' if name in TARGETS else 'distractor',
            'position': [round(value, 4) for value in positions[name]],
            'inside_bowl': inside_bowl(positions[name]),
        }
        for name in OBJECTS
    }
    success = all(objects[name]['inside_bowl'] for name in TARGETS) and all(
        not objects[name]['inside_bowl'] for name in DISTRACTORS
    )
    return {'success': success, 'objects': objects}


def public_grade(private: dict[str, object]) -> dict[str, object]:
    objects = private['objects']
    return {
        'task_success': private['success'],
        'criteria': {
            'repeated_color_cubes_in_bowl': {
                'placed': sum(bool(objects[name]['inside_bowl']) for name in TARGETS),
                'required': len(TARGETS),
            },
            'unique_color_cubes_in_bowl': sum(
                bool(objects[name]['inside_bowl']) for name in DISTRACTORS
            ),
        },
    }


def read_positions(topic: str, timeout: float) -> dict[str, tuple[float, float, float]]:
    positions: dict[str, tuple[float, float, float]] = {}
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
        while not all(name in positions for name in OBJECTS):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            try:
                packet = packets.get(timeout=remaining)
            except Empty:
                break
            positions.update(
                (name, position)
                for name, position in _decode_pose_vector(packet).items()
                if name in OBJECTS
            )
        missing = [name for name in OBJECTS if name not in positions]
        if missing:
            raise RuntimeError(f'timed out waiting for poses: {", ".join(missing)}')
        return positions
    finally:
        node.unsubscribe(topic)
