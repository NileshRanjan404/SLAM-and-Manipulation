import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


 
H0 = 0.075   # base_link to shoulder_lift pivot height
L1 = 0.2     # shoulder_lift to elbow (upper_arm_link)
L2 = 0.25    # elbow to wrist (forearm_link)
L3 = 0.175   # wrist to end_effector_link


def solve_ik(x, y, z, elbow_up=True):
     
    # solution
    theta1 = math.atan2(y, x)

    #Horizontal distance from axis to target
    r = math.hypot(x, y)
    z_prime = z - H0 # target height above the shoulder pivot

    # wrist is horizontal
    r_w = r - L3
    z_w = z_prime

    D = math.hypot(r_w, z_w)

    # reachable range 
    if D > (L1 + L2) or D < abs(L1 - L2):
        return None

    # TRIGO calculations
    cos_gamma = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))  

    gamma_mag = math.acos(cos_gamma)
    gamma = -gamma_mag if elbow_up else gamma_mag

    beta1 = math.atan2(z_w, r_w) - math.atan2(
        L2 * math.sin(gamma), L1 + L2 * math.cos(gamma)
    )

    
    theta2 = math.pi / 2 - beta1
    theta3 = -gamma
    theta4 = beta1 + gamma

    return theta1, theta2, theta3, theta4


class CustomIKNode(Node):
    def __init__(self, target_xyz):
        super().__init__('custom_ik_node')
        self.target_xyz = target_xyz

        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_joint',
        ]

    def send_trajectory(self):
        x, y, z = self.target_xyz
        self.get_logger().info(f'Target: x={x}, y={y}, z={z}')

        solution = solve_ik(x, y, z)
        if solution is None:
            self.get_logger().error('Target is UNREACHABLE for this arm geometry.')
            rclpy.shutdown()
            return

        theta1, theta2, theta3, theta4 = solution
        self.get_logger().info(
            f'Computed joint angles (rad): '
            f'shoulder_pan={theta1:.4f}, shoulder_lift={theta2:.4f}, '
            f'elbow={theta3:.4f}, wrist={theta4:.4f}'
        )

        self.get_logger().info('Waiting for arm_controller action server...')
        self._action_client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [theta1, theta2, theta3, theta4]
        point.time_from_start.sec = 4  # 4 seconds to reach the pose smoothly

        goal_msg.trajectory.points = [point]

        self.get_logger().info('Publishing trajectory to arm_controller...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal REJECTED by controller.')
            rclpy.shutdown()
            return
        self.get_logger().info('Trajectory goal ACCEPTED. Executing...')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Trajectory execution finished. Error code: {result.error_code}')
        if result.error_code == 0:
            self.get_logger().info('Arm successfully reached the target coordinate!')
        else:
            self.get_logger().warn('Trajectory finished with a non-zero error code.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # EDIT THIS:  
    target = (0.25, 0.1, 0.15)

    if len(sys.argv) == 4:
        target = tuple(float(v) for v in sys.argv[1:4])

    node = CustomIKNode(target)
    node.send_trajectory()
    rclpy.spin(node)


if __name__ == '__main__':
    main()