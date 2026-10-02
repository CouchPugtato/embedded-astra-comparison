"""Reset the task without restarting Gazebo or its controller manager."""

import sys
import time
from xml.etree import ElementTree

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from controller_manager_msgs.srv import ListControllers
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node as GazeboNode
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from grading import INITIAL_POSITIONS, OBJECTS, read_positions
from manipulation_api import Robot


CONTROLLERS = ('panda_arm_controller', 'panda_hand_controller')
REQUIRED_CONTROLLERS = ('joint_state_broadcaster',) + CONTROLLERS
ARM_JOINTS = tuple(f'panda_joint{number}' for number in range(1, 8))
INITIAL_JOINTS = {
    'panda_joint1': 0.0,
    'panda_joint2': -0.785,
    'panda_joint3': 0.0,
    'panda_joint4': -2.356,
    'panda_joint5': 0.0,
    'panda_joint6': 1.571,
    'panda_joint7': 0.785,
    'panda_finger_joint1': 0.035,
    'panda_finger_joint2': 0.035,
}


class ResetError(RuntimeError):
    pass


def _progress(message: str) -> None:
    print(f'[reset] {message}', file=sys.stderr, flush=True)


def _wait_controllers_active(timeout: float) -> None:
    owns_rclpy = not rclpy.ok()
    if owns_rclpy:
        rclpy.init()
    node = Node('scene_controller_wait')
    try:
        client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        if not client.wait_for_service(timeout_sec=timeout):
            raise ResetError('controller list service is unavailable')
        deadline = time.monotonic() + timeout
        states = {}
        while time.monotonic() < deadline:
            future = client.call_async(ListControllers.Request())
            remaining = deadline - time.monotonic()
            rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.0, remaining))
            response = future.result() if future.done() else None
            states = {} if response is None else {
                controller.name: controller.state for controller in response.controller
            }
            if all(states.get(name) == 'active' for name in REQUIRED_CONTROLLERS):
                return
            time.sleep(0.1)
        missing = [name for name in REQUIRED_CONTROLLERS if states.get(name) != 'active']
        raise ResetError(f'controllers did not become active: {", ".join(missing)}')
    finally:
        node.destroy_node()
        if owns_rclpy and rclpy.ok():
            rclpy.shutdown()


def _wait_future(node: Node, future: object, deadline: float, description: str):
    while not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise ResetError(f'timed out waiting for {description}')
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    exception = future.exception()
    if exception is not None:
        raise ResetError(f'{description} failed: {exception}') from exception
    return future.result()


def _send_action(node: Node, client: ActionClient, goal: object, deadline: float, name: str):
    remaining = deadline - time.monotonic()
    if remaining <= 0.0 or not client.wait_for_server(timeout_sec=remaining):
        raise ResetError(f'{name} action is unavailable')
    handle = _wait_future(
        node, client.send_goal_async(goal), deadline, f'{name} goal acceptance'
    )
    if not handle.accepted:
        raise ResetError(f'{name} controller rejected its reset goal')
    return _wait_future(node, handle.get_result_async(), deadline, name).result


def _home_robot(timeout: float) -> None:
    owns_rclpy = not rclpy.ok()
    if owns_rclpy:
        rclpy.init()
    node = Node('scene_robot_home')
    deadline = time.monotonic() + timeout
    try:
        gripper_client = ActionClient(
            node, ParallelGripperCommand, '/panda_hand_controller/gripper_cmd'
        )
        gripper_goal = ParallelGripperCommand.Goal()
        gripper_goal.command.name = ['panda_finger_joint1']
        gripper_goal.command.position = [INITIAL_JOINTS['panda_finger_joint1']]
        gripper_goal.command.effort = [20.0]
        gripper_result = _send_action(
            node, gripper_client, gripper_goal, deadline, 'gripper'
        )
        if not (gripper_result.reached_goal or gripper_result.stalled):
            raise ResetError('gripper did not open for reset')

        arm_client = ActionClient(
            node, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory'
        )
        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [INITIAL_JOINTS[name] for name in ARM_JOINTS]
        point.time_from_start.sec = 3
        arm_goal.trajectory.points = [point]
        arm_result = _send_action(node, arm_client, arm_goal, deadline, 'arm home')
        if arm_result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise ResetError(
                f'arm homing failed ({arm_result.error_code}): {arm_result.error_string}'
            )
    finally:
        node.destroy_node()
        if owns_rclpy and rclpy.ok():
            rclpy.shutdown()


def _object_models() -> dict[str, str]:
    world_path = (
        get_package_share_directory('tabletop_sim') + '/worlds/tabletop.sdf'
    )
    world = ElementTree.parse(world_path).getroot().find('world')
    if world is None:
        raise ResetError(f'no world element in {world_path}')
    models = {
        model.get('name'): model
        for model in world.findall('model')
        if model.get('name') in OBJECTS
    }
    missing = [name for name in OBJECTS if name not in models]
    if missing:
        raise ResetError(f'missing object definitions: {", ".join(missing)}')
    return {
        name: f'<sdf version="1.9">{ElementTree.tostring(models[name], encoding="unicode")}</sdf>'
        for name in OBJECTS
    }


def _gazebo_request(
    node: GazeboNode,
    service: str,
    request: object,
    request_type: type,
    timeout: float,
    *,
    require_success: bool = True,
) -> None:
    executed, response = node.request(
        service, request, request_type, Boolean, int(timeout * 1000)
    )
    if not executed or (require_success and not response.data):
        raise ResetError(f'Gazebo service failed or timed out: {service}')


def _set_paused(node: GazeboNode, paused: bool, timeout: float) -> None:
    request = WorldControl()
    request.pause = paused
    _gazebo_request(
        node, '/world/tabletop/control', request, WorldControl, timeout
    )


def _replace_objects(timeout: float) -> None:
    models = _object_models()
    node = GazeboNode()
    _set_paused(node, True, timeout)
    try:
        for name in OBJECTS:
            request = Entity(name=name, type=Entity.MODEL)
            _gazebo_request(
                node,
                '/world/tabletop/remove/blocking',
                request,
                Entity,
                timeout,
                require_success=False,
            )
        for name in OBJECTS:
            request = EntityFactory()
            request.sdf = models[name]
            request.allow_renaming = False
            _gazebo_request(
                node,
                '/world/tabletop/create/blocking',
                request,
                EntityFactory,
                timeout,
            )
    finally:
        _set_paused(node, False, timeout)


def _check_ros(timeout: float) -> None:
    owns_rclpy = not rclpy.ok()
    if owns_rclpy:
        rclpy.init()
    node = Node('scene_readiness_check')
    latest_joints: dict[str, tuple[float, float]] = {}

    def on_joints(message: JointState) -> None:
        velocities = dict(zip(message.name, message.velocity))
        latest_joints.clear()
        latest_joints.update(
            (name, (position, velocities.get(name, 0.0)))
            for name, position in zip(message.name, message.position)
        )

    try:
        controller_client = node.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        if not controller_client.wait_for_service(timeout_sec=timeout):
            raise ResetError('controller list service is unavailable')
        future = controller_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        response = future.result() if future.done() else None
        states = {} if response is None else {
            controller.name: controller.state for controller in response.controller
        }
        inactive = [name for name in REQUIRED_CONTROLLERS if states.get(name) != 'active']
        if inactive:
            raise ResetError(f'controllers are not active: {", ".join(inactive)}')

        move_group = ActionClient(node, MoveGroup, '/move_action')
        if not move_group.wait_for_server(timeout_sec=timeout):
            raise ResetError('MoveIt /move_action is unavailable')
        gripper = ActionClient(
            node, ParallelGripperCommand, '/panda_hand_controller/gripper_cmd'
        )
        if not gripper.wait_for_server(timeout_sec=timeout):
            raise ResetError('parallel gripper action is unavailable')

        node.create_subscription(
            JointState, '/joint_states', on_joints, qos_profile_sensor_data
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(name in latest_joints for name in INITIAL_JOINTS):
                position_error = max(
                    abs(latest_joints[name][0] - expected)
                    for name, expected in INITIAL_JOINTS.items()
                )
                max_velocity = max(abs(latest_joints[name][1]) for name in INITIAL_JOINTS)
                if position_error <= 0.03 and max_velocity <= 0.05:
                    return
        raise ResetError('robot did not settle at its initial joint state')
    finally:
        node.destroy_node()
        if owns_rclpy and rclpy.ok():
            rclpy.shutdown()


def reset_and_verify(seed: int, timeout: float = 30.0, settle_seconds: float = 1.0) -> dict:
    if not 0 <= seed <= 0xFFFFFFFF:
        raise ValueError('seed must be between 0 and 4294967295')
    if timeout <= 0.0 or settle_seconds < 0.0:
        raise ValueError('timeout must be positive and settle time cannot be negative')

    _progress('checking controllers')
    _wait_controllers_active(timeout)
    _progress('opening the gripper and homing the arm')
    _home_robot(timeout)
    _progress('recreating task objects')
    _replace_objects(timeout)
    _progress('waiting for the scene to settle')
    time.sleep(settle_seconds)
    _progress('verifying controllers and robot joints')
    _check_ros(timeout)
    _progress('verifying object poses')
    positions = read_positions('/world/tabletop/pose/info', timeout)
    misplaced = [
        name
        for name, expected in INITIAL_POSITIONS.items()
        if max(abs(actual - target) for actual, target in zip(positions[name], expected)) > 0.012
    ]
    if misplaced:
        raise ResetError(f'objects did not reset: {", ".join(misplaced)}')

    _progress('verifying camera and TF')
    with Robot() as robot:
        frame = robot.capture_camera(timeout)
        robot.get_end_effector_pose(timeout)
    _progress('ready')
    return {
        'seed': seed,
        'controllers_ready': True,
        'move_group_ready': True,
        'gripper_ready': True,
        'robot_at_initial_joints': True,
        'objects_at_initial_poses': True,
        'camera': f'{frame.width}x{frame.height}',
        'tf_ready': True,
    }
