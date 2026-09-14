import time
import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion, Wrench
from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode


class SimplePoseTargetPolicy(Policy):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("SimplePoseTargetPolicy initialized")

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        **kwargs,
    ):
        self.get_logger().info("Sending single pose target...")

        # 🎯 Target pose
        pose = Pose(
            position=Point(x=-0.3, y=0.0, z=0.2),  # safer test
            orientation=Quaternion(
                x=1.0,
                y=0.0,
                z=0.0,
                w=0.0,
            ),
        )

        motion_update = MotionUpdate()
        motion_update.header.frame_id = "base_link"
        motion_update.header.stamp = self.get_clock().now().to_msg()

        motion_update.pose = pose

        # stiffness & damping
        motion_update.target_stiffness = np.diag([85.0]*6).flatten()
        motion_update.target_damping = np.diag([75.0]*6).flatten()

        motion_update.feedforward_wrench_at_tip = Wrench(
            force=Point(x=0.0, y=0.0, z=0.0),
            torque=Point(x=0.0, y=0.0, z=0.0),
        )

        motion_update.wrench_feedback_gains_at_tip = [0.0]*6

        # ✅ FIXED HERE
        motion_update.trajectory_generation_mode.mode = (
            TrajectoryGenerationMode.MODE_POSITION
        )

        # 🚀 send once
        move_robot(motion_update=motion_update)

        send_feedback("Pose sent")

        time.sleep(5)

        self.get_logger().info("Done")
        return True
