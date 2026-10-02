#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class SceneSetup(Node):
    """Install the tabletop and reset-position tray in MoveIt's planning scene."""

    def __init__(self) -> None:
        super().__init__('planning_scene_setup')
        self._client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

    @staticmethod
    def _box(
        object_id: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
    ) -> CollisionObject:
        item = CollisionObject()
        item.header.frame_id = 'world'
        item.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.0
        item.primitives.append(primitive)
        item.primitive_poses.append(pose)
        item.operation = CollisionObject.ADD
        return item

    def apply(self, timeout: float = 30.0) -> bool:
        if not self._client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('Timed out waiting for /apply_planning_scene')
            return False

        scene = PlanningScene()
        scene.is_diff = True
        # The collision surface starts beyond the robot pedestal so it does not
        # intersect panda_link0 while still protecting the manipulation area.
        scene.world.collision_objects = [
            self._box('table_work_surface', (1.0, 0.9, 0.08), (0.65, 0.0, -0.045)),
            self._box('blue_tray_base', (0.22, 0.18, 0.024), (0.52, -0.22, 0.012)),
            self._box('blue_tray_rim_x_positive', (0.01, 0.18, 0.05), (0.625, -0.22, 0.047)),
            self._box('blue_tray_rim_x_negative', (0.01, 0.18, 0.05), (0.415, -0.22, 0.047)),
            self._box('blue_tray_rim_y_positive', (0.22, 0.01, 0.05), (0.52, -0.135, 0.047)),
            self._box('blue_tray_rim_y_negative', (0.22, 0.01, 0.05), (0.52, -0.305, 0.047)),
        ]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return bool(future.done() and future.result() and future.result().success)


def main() -> None:
    rclpy.init()
    node = SceneSetup()
    success = node.apply()
    if success:
        node.get_logger().info('Applied tabletop collision scene')
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
