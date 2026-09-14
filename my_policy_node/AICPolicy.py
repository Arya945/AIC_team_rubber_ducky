import rclpy
from rclpy.node import Node

import numpy as np

from aic_interfaces.msg import Observation   # adjust if needed
from std_msgs.msg import Float64MultiArray   # or correct type


class AICPolicy(Node):

    def __init__(self):
        super().__init__('aic_policy')

        # -------------------------
        # Subscriber (INPUT)
        # -------------------------
        self.sub = self.create_subscription(
            Observation,
            '/observations',
            self.obs_callback,
            10
        )

        # -------------------------
        # Publisher (OUTPUT)
        # -------------------------
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/aic_controller/joint_commands',
            10
        )

        self.get_logger().info("Policy node started")

    # ---------------------------------
    # MAIN POLICY CALLBACK
    # ---------------------------------
    def obs_callback(self, obs):

        # -------- DEBUG --------
        self.get_logger().info("Received observation")

        # Extract data
        joint_pos = np.array(obs.joint_states.position)

        tcp = obs.controller_state.tcp_pose.position
        tcp_pos = np.array([tcp.x, tcp.y, tcp.z])

        self.get_logger().info(f"TCP: {tcp_pos}")

        # ---------------------------------
        # POLICY LOGIC (your brain)
        # ---------------------------------

        # Example: hold position
        action = joint_pos * 0.0

        # ---------------------------------
        # Publish action
        # ---------------------------------
        msg = Float64MultiArray()
        msg.data = action.tolist()

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = AICPolicy()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
