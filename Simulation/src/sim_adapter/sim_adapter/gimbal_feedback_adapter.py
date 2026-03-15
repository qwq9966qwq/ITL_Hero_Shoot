"""Gimbal feedback adapter: convert rmoss Gimbal (IMU world angles) to standard JointState."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# rmoss_interfaces is simulation-only; import at runtime
from rmoss_interfaces.msg import Gimbal


class GimbalFeedbackAdapter(Node):
    """Subscribe to gimbal_state (rmoss Gimbal, IMU world yaw/pitch),
    publish as gimbal_world_feedback (JointState) so lob_shot_manager
    can do convergence checks without depending on rmoss_interfaces.
    """

    def __init__(self):
        super().__init__('gimbal_feedback_adapter')

        self.sub = self.create_subscription(
            Gimbal,
            'robot_base/gimbal_state',
            self.callback,
            10,
        )
        self.pub = self.create_publisher(
            JointState,
            'gimbal_world_feedback',
            10,
        )
        self.get_logger().info('Gimbal feedback adapter started (Gimbal -> JointState world angles)')

    def callback(self, msg: Gimbal):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = ['gimbal_yaw_joint', 'gimbal_pitch_joint']
        out.position = [float(msg.yaw), float(msg.pitch)]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalFeedbackAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
