"""Joint state adapter: convert simulation dual-yaw to simplified single-yaw."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class JointStateAdapter(Node):
    """Subscribe to simulation joint_states (dual-yaw), publish adapted joint_states (single-yaw).

    Conversion:
        simplified gimbal_yaw_joint  = sim gimbal_yaw_odom_joint + sim gimbal_yaw_joint
        simplified gimbal_pitch_joint = sim gimbal_pitch_joint  (pass-through, odom is fixed=0)
    """

    def __init__(self):
        super().__init__('joint_state_adapter')

        self.sub = self.create_subscription(
            JointState,
            'joint_states',
            self.callback,
            qos_profile_sensor_data,
        )
        self.pub = self.create_publisher(
            JointState,
            'adapted_joint_states',
            10,
        )
        self.get_logger().info('Joint state adapter started (dual-yaw -> single-yaw)')

    def callback(self, msg: JointState):
        yaw_odom_pos = 0.0
        yaw_odom_vel = 0.0
        yaw_pos = None
        yaw_vel = 0.0
        pitch_pos = None
        pitch_vel = 0.0

        for i, name in enumerate(msg.name):
            if name == 'gimbal_yaw_odom_joint':
                yaw_odom_pos = msg.position[i] if msg.position else 0.0
                yaw_odom_vel = msg.velocity[i] if msg.velocity else 0.0
            elif name == 'gimbal_yaw_joint':
                yaw_pos = msg.position[i] if msg.position else 0.0
                yaw_vel = msg.velocity[i] if msg.velocity else 0.0
            elif name == 'gimbal_pitch_joint':
                pitch_pos = msg.position[i] if msg.position else 0.0
                pitch_vel = msg.velocity[i] if msg.velocity else 0.0

        if yaw_pos is None or pitch_pos is None:
            return

        out = JointState()
        out.header = msg.header
        out.name = ['gimbal_yaw_joint', 'gimbal_pitch_joint']
        out.position = [yaw_odom_pos + yaw_pos, pitch_pos]
        out.velocity = [yaw_odom_vel + yaw_vel, pitch_vel]

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
