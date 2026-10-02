from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Iterable

import rclpy
from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import Pose as PoseMsg
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, MoveItErrorCodes, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


# Public data types and validation


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    @property
    def position(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    @property
    def orientation(self) -> tuple[float, float, float, float]:
        return (self.qx, self.qy, self.qz, self.qw)


@dataclass(frozen=True)
class CameraFrame:
    width: int
    height: int
    rgb: bytes
    timestamp_ns: int

    def pixel(self, x: int, y: int) -> tuple[int, int, int]:
        if not 0 <= x < self.width or not 0 <= y < self.height:
            raise IndexError(f'pixel ({x}, {y}) is outside {self.width}x{self.height}')
        offset = (y * self.width + x) * 3
        return self.rgb[offset], self.rgb[offset + 1], self.rgb[offset + 2]


@dataclass(frozen=True)
class WorkspaceLimits:
    x_min: float = 0.15
    x_max: float = 0.80
    y_min: float = -0.55
    y_max: float = 0.55
    z_min: float = 0.04
    z_max: float = 1.10

    def validate(self, position: Iterable[float]) -> tuple[float, float, float]:
        values = tuple(float(value) for value in position)
        if len(values) != 3:
            raise ValueError('position must contain exactly three values')
        x, y, z = values
        if not (self.x_min <= x <= self.x_max):
            raise ValueError(f'x={x:.3f} is outside [{self.x_min}, {self.x_max}]')
        if not (self.y_min <= y <= self.y_max):
            raise ValueError(f'y={y:.3f} is outside [{self.y_min}, {self.y_max}]')
        if not (self.z_min <= z <= self.z_max):
            raise ValueError(f'z={z:.3f} is outside [{self.z_min}, {self.z_max}]')
        return x, y, z


class RobotError(RuntimeError):
    pass


# ROS-backed control API


class Robot:
    """Stable policy-facing API backed by MoveIt and ros2_control actions."""

    # ROS lifecycle

    def __init__(
        self,
        *,
        workspace: WorkspaceLimits | None = None,
        command_timeout: float = 45.0,
        planning_timeout: float = 8.0,
        velocity_scaling: float = 0.25,
        acceleration_scaling: float = 0.25,
        end_effector_frame: str = 'panda_link8',
    ) -> None:
        if not 0.0 < velocity_scaling <= 1.0 or not 0.0 < acceleration_scaling <= 1.0:
            raise ValueError('velocity and acceleration scaling must be in (0, 1]')
        self.workspace = workspace or WorkspaceLimits()
        self.command_timeout = float(command_timeout)
        self.planning_timeout = float(planning_timeout)
        self.velocity_scaling = float(velocity_scaling)
        self.acceleration_scaling = float(acceleration_scaling)
        self.end_effector_frame = end_effector_frame
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self._node = Node('manipulation_api')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node, spin_thread=False)
        self._move_group = ActionClient(self._node, MoveGroup, '/move_action')
        self._gripper = ActionClient(
            self._node, ParallelGripperCommand, '/panda_hand_controller/gripper_cmd'
        )
        self._camera_condition = threading.Condition()
        self._camera_frame: CameraFrame | None = None
        self._camera_sequence = 0
        self._camera_error: str | None = None
        self._camera_subscription = self._node.create_subscription(
            Image,
            '/camera/image_raw',
            self._camera_callback,
            qos_profile_sensor_data,
        )
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def __enter__(self) -> Robot:
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def close(self) -> None:
        if not hasattr(self, '_node'):
            return
        self._executor.shutdown(timeout_sec=2.0)
        self._spin_thread.join(timeout=2.0)
        self._executor.remove_node(self._node)
        self._node.destroy_node()
        del self._node
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()

    # Shared helpers

    @staticmethod
    def _normalize_quaternion(values: Iterable[float]) -> tuple[float, float, float, float]:
        q = tuple(float(value) for value in values)
        if len(q) != 4:
            raise ValueError('orientation must contain exactly four values')
        magnitude = math.sqrt(sum(value * value for value in q))
        if magnitude < 1e-9:
            raise ValueError('orientation quaternion cannot be zero')
        return tuple(value / magnitude for value in q)  # type: ignore[return-value]

    @staticmethod
    def _wait_future(future: object, timeout: float, description: str):
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout):
            raise RobotError(f'timed out waiting for {description}')
        exception = future.exception()
        if exception is not None:
            raise RobotError(f'{description} failed: {exception}') from exception
        return future.result()

    # Perception

    @staticmethod
    def _image_to_rgb(message: Image) -> bytes:
        layouts = {
            'rgb8': (3, (0, 1, 2)),
            'bgr8': (3, (2, 1, 0)),
            'rgba8': (4, (0, 1, 2)),
            'bgra8': (4, (2, 1, 0)),
        }
        encoding = message.encoding.lower()
        if encoding not in layouts:
            raise ValueError(f'unsupported camera encoding {message.encoding!r}')
        channels, order = layouts[encoding]
        row_bytes = message.width * channels
        if message.step < row_bytes or len(message.data) < message.step * message.height:
            raise ValueError('camera image data is shorter than its dimensions require')

        source = memoryview(message.data)
        if encoding == 'rgb8' and message.step == row_bytes:
            return bytes(source[: row_bytes * message.height])

        rgb = bytearray(message.width * message.height * 3)
        output = 0
        for y in range(message.height):
            row = y * message.step
            for x in range(message.width):
                pixel = row + x * channels
                rgb[output] = source[pixel + order[0]]
                rgb[output + 1] = source[pixel + order[1]]
                rgb[output + 2] = source[pixel + order[2]]
                output += 3
        return bytes(rgb)

    def _camera_callback(self, message: Image) -> None:
        try:
            frame = CameraFrame(
                width=message.width,
                height=message.height,
                rgb=self._image_to_rgb(message),
                timestamp_ns=message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec,
            )
            error = None
        except ValueError as exception:
            frame = None
            error = str(exception)

        with self._camera_condition:
            self._camera_frame = frame
            self._camera_error = error
            self._camera_sequence += 1
            self._camera_condition.notify_all()

    def capture_camera(self, timeout: float = 5.0) -> CameraFrame:
        if timeout <= 0.0:
            raise ValueError('timeout must be positive')
        deadline = time.monotonic() + timeout
        with self._camera_condition:
            starting_sequence = self._camera_sequence
            while self._camera_sequence == starting_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RobotError('no RGB camera frame received from /camera/image_raw')
                self._camera_condition.wait(remaining)
            if self._camera_error is not None:
                raise RobotError(self._camera_error)
            if self._camera_frame is None:
                raise RobotError('RGB camera did not produce a usable frame')
            return self._camera_frame

    def get_end_effector_pose(self, timeout: float = 5.0) -> Pose:
        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    'world', self.end_effector_frame, Time()
                ).transform
                return Pose(
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                )
            except TransformException as error:
                last_error = error
                time.sleep(0.05)
        raise RobotError(f'no world -> {self.end_effector_frame} transform: {last_error}')

    # Arm movement

    def move_to(
        self,
        position: Iterable[float],
        orientation: Iterable[float] | None = None,
    ) -> Pose:
        target_x, target_y, target_z = self.workspace.validate(position)
        if orientation is None:
            orientation = self.get_end_effector_pose().orientation
        qx, qy, qz, qw = self._normalize_quaternion(orientation)

        if not self._move_group.wait_for_server(timeout_sec=10.0):
            raise RobotError('MoveIt /move_action is unavailable')

        target = PoseMsg()
        target.position.x, target.position.y, target.position.z = target_x, target_y, target_z
        target.orientation.x, target.orientation.y = qx, qy
        target.orientation.z, target.orientation.w = qz, qw

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'world'
        position_constraint.link_name = self.end_effector_frame
        position_constraint.weight = 1.0
        tolerance = SolidPrimitive()
        tolerance.type = SolidPrimitive.SPHERE
        tolerance.dimensions = [0.003]
        position_constraint.constraint_region.primitives.append(tolerance)
        position_constraint.constraint_region.primitive_poses.append(target)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'world'
        orientation_constraint.link_name = self.end_effector_frame
        orientation_constraint.orientation = target.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.03
        orientation_constraint.absolute_y_axis_tolerance = 0.03
        orientation_constraint.absolute_z_axis_tolerance = 0.03
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = 'end_effector_pose_goal'
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = 'panda_arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = self.planning_timeout
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.acceleration_scaling
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints.append(constraints)
        goal.request.workspace_parameters.header.frame_id = 'world'
        goal.request.workspace_parameters.min_corner.x = self.workspace.x_min
        goal.request.workspace_parameters.min_corner.y = self.workspace.y_min
        goal.request.workspace_parameters.min_corner.z = self.workspace.z_min
        goal.request.workspace_parameters.max_corner.x = self.workspace.x_max
        goal.request.workspace_parameters.max_corner.y = self.workspace.y_max
        goal.request.workspace_parameters.max_corner.z = self.workspace.z_max
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        goal_handle = self._wait_future(
            self._move_group.send_goal_async(goal), 10.0, 'MoveIt goal acceptance'
        )
        if not goal_handle.accepted:
            raise RobotError('MoveIt rejected the pose goal')
        wrapped_result = self._wait_future(
            goal_handle.get_result_async(), self.command_timeout, 'MoveIt execution'
        )
        error_code = wrapped_result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            raise RobotError(f'MoveIt failed with error code {error_code}')
        return self.get_end_effector_pose()

    def move_relative(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> Pose:
        current = self.get_end_effector_pose()
        return self.move_to(
            position=(current.x + dx, current.y + dy, current.z + dz),
            orientation=current.orientation,
        )

    # Gripper control

    def _command_gripper(self, position: float, max_effort: float = 35.0) -> None:
        if not 0.0 <= position <= 0.04:
            raise ValueError('gripper position must be in [0.0, 0.04] metres')
        if not self._gripper.wait_for_server(timeout_sec=10.0):
            raise RobotError('gripper action server is unavailable')
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ['panda_finger_joint1']
        goal.command.position = [position]
        goal.command.effort = [max_effort]
        goal_handle = self._wait_future(
            self._gripper.send_goal_async(goal), 10.0, 'gripper goal acceptance'
        )
        if not goal_handle.accepted:
            raise RobotError('gripper controller rejected the command')
        wrapped_result = self._wait_future(
            goal_handle.get_result_async(), self.command_timeout, 'gripper command'
        )
        if not (wrapped_result.result.reached_goal or wrapped_result.result.stalled):
            raise RobotError('gripper neither reached its goal nor stalled on an object')

    def open_gripper(self) -> None:
        self._command_gripper(0.035, max_effort=20.0)

    def close_gripper(self) -> None:
        # Command through the 5 cm cube's contact point; the controller treats
        # a stable effort-limited stall as a successful grasp.
        self._command_gripper(0.020, max_effort=35.0)
