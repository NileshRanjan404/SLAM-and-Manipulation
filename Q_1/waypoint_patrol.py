import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped


def make_pose(x, y, theta, frame_id='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(theta / 2.0)
    pose.pose.orientation.w = math.cos(theta / 2.0)
    return pose


class WaypointPatrol(Node):
    def __init__(self):
        super().__init__('waypoint_patrol')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        
        self.waypoints = [
            make_pose(5.34, 1.88, 0.0),
            make_pose(4.45, -4.15, 1.57),
            make_pose(-0.10, -8.27, -1.57)
        ]
        self.last_wp = -1

    def send_goal(self):
        self.get_logger().info('Waiting for follow_waypoints action server...')
        self._action_client.wait_for_server()

        for wp in self.waypoints:
            wp.header.stamp = self.get_clock().now().to_msg()

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints

        self.get_logger().info('Navigating to Waypoint 1...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted by server')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        current_wp = feedback_msg.feedback.current_waypoint
        if current_wp != self.last_wp:
            if self.last_wp >= 0:
                self.get_logger().info(f'Waypoint {self.last_wp + 1} Reached!')
            if current_wp < len(self.waypoints):
                self.get_logger().info(f'Navigating to Waypoint {current_wp + 1}...')
            self.last_wp = current_wp

    def result_callback(self, future):
        result = future.result().result
        if result.missed_waypoints:
            self.get_logger().warn(f'Missed waypoints: {list(result.missed_waypoints)}')
        else:
            self.get_logger().info(f'Waypoint {len(self.waypoints)} Reached! Patrol complete.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrol()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
