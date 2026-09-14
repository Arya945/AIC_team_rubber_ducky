import time
import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion, Wrench
from aic_model.policy import (
    Policy,
    GetObservationCallback,
    MoveRobotCallback,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task
from aic_control_interfaces.msg import (
    MotionUpdate,
    JointMotionUpdate,
    TrajectoryGenerationMode,
)


class HybridFeedbackPolicy(Policy):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("HybridFeedbackPolicy initialized")

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        **kwargs,
    ):
        self.get_logger().info("=== START HYBRID CONTROL ===")

        # ================================
        # 🎯 TARGETS
        # ================================
        cartesian_target1 = np.array([-0.3842129127757932, 0.2480714513839817, 0.30])
        joint_target1 = np.array([-0.35, -1.65, -1.57, -1.57, 2.0, 0.9])
        cartesian_target2 = np.array([-0.3842129127757932, 0.2480714513839817, 0.26])
        joint_target2 = np.array([-0.4, -1.65, -1.57, -1.57, 2.0, 0.9])
        cartesian_target3 = np.array([-0.3818283598790449, 0.2122682069918802, 0.22753940157239446])
        
        cartesian_threshold = 0.01   # 1 cm
        joint_threshold = 0.05       # rad
        # ================================
        # 1 CARTESIAN CONTROL LOOP
        # ================================
        self.get_logger().info("Phase 1: Cartesian Control")

        motion = MotionUpdate()
        motion.header.frame_id = "base_link"

        motion.pose = Pose(
            position=Point(
                x=cartesian_target1[0],
                y=cartesian_target1[1],
                z=cartesian_target1[2],
            ),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
        )

        motion.target_stiffness = np.diag([85.0]*6).flatten()
        motion.target_damping = np.diag([75.0]*6).flatten()
        motion.feedforward_wrench_at_tip = Wrench()
        motion.wrench_feedback_gains_at_tip = [0.0]*6

        motion.trajectory_generation_mode.mode = (
            TrajectoryGenerationMode.MODE_POSITION
        )

        while True:
            obs = get_observation()
            if obs is None:
                continue

            tcp = obs.controller_state.tcp_pose.position

            current_pos = np.array([tcp.x, tcp.y, tcp.z])
            error = np.linalg.norm(current_pos - cartesian_target1)

            self.get_logger().info(f"[Cartesian] Error: {error:.4f}")

            if error < cartesian_threshold:
                self.get_logger().info("Cartesian target reached ✅")
                break

            motion.header.stamp = self.get_clock().now().to_msg()
            move_robot(motion_update=motion)

            send_feedback(f"Cartesian moving... error={error:.4f}")
            time.sleep(0.1)
        # ================================
        # 2 JOINT CONTROL LOOP
        # ================================
        self.get_logger().info("Phase 2: Joint Control")

        joint_cmd = JointMotionUpdate()
        joint_cmd.target_state.positions = joint_target1.tolist()

        joint_cmd.target_stiffness = [85.0]*6
        joint_cmd.target_damping = [75.0]*6

        joint_cmd.trajectory_generation_mode.mode = (
            TrajectoryGenerationMode.MODE_POSITION
        )

        while True:
            obs = get_observation()
            if obs is None:
                continue

            joints = np.array(obs.joint_states.position[:6])
            error = np.linalg.norm(joints - joint_target1)

            self.get_logger().info(f"[Joint] Error: {error:.4f}")

            if error < joint_threshold:
                self.get_logger().info("Joint target reached ✅")
                break

            move_robot(joint_motion_update=joint_cmd)

            send_feedback(f"Joint moving... error={error:.4f}")
            time.sleep(0.1)
        # ================================
        # 3 CARTESIAN CONTROL LOOP
        # ================================
        self.get_logger().info("Phase 1: Cartesian Control")

        motion = MotionUpdate()
        motion.header.frame_id = "base_link"

        motion.pose = Pose(
            position=Point(
                x=cartesian_target2[0],
                y=cartesian_target2[1],
                z=cartesian_target2[2],
            ),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
        )

        motion.target_stiffness = np.diag([85.0]*6).flatten()
        motion.target_damping = np.diag([75.0]*6).flatten()
        motion.feedforward_wrench_at_tip = Wrench()
        motion.wrench_feedback_gains_at_tip = [0.0]*6

        motion.trajectory_generation_mode.mode = (
            TrajectoryGenerationMode.MODE_POSITION
        )

        while True:
            obs = get_observation()
            if obs is None:
                continue

            tcp = obs.controller_state.tcp_pose.position

            current_pos = np.array([tcp.x, tcp.y, tcp.z])
            error = np.linalg.norm(current_pos - cartesian_target2)

            self.get_logger().info(f"[Cartesian] Error: {error:.4f}")

            if error < cartesian_threshold:
                self.get_logger().info("Cartesian target reached ✅")
                break

            motion.header.stamp = self.get_clock().now().to_msg()
            move_robot(motion_update=motion)

            send_feedback(f"Cartesian moving... error={error:.4f}")
            time.sleep(0.1)
        # ================================
        # 4 JOINT CONTROL LOOP
        # ================================
        self.get_logger().info("Phase 2: Joint Control")

        joint_cmd = JointMotionUpdate()
        joint_cmd.target_state.positions = joint_target2.tolist()

        joint_cmd.target_stiffness = [85.0]*6
        joint_cmd.target_damping = [75.0]*6

        joint_cmd.trajectory_generation_mode.mode = (
            TrajectoryGenerationMode.MODE_POSITION
        )

        while True:
            obs = get_observation()
            if obs is None:
                continue

            joints = np.array(obs.joint_states.position[:6])
            error = np.linalg.norm(joints - joint_target2)

            self.get_logger().info(f"[Joint] Error: {error:.4f}")

            if error < joint_threshold:
                self.get_logger().info("Joint target reached ✅")
                break

            move_robot(joint_motion_update=joint_cmd)

            send_feedback(f"Joint moving... error={error:.4f}")
            time.sleep(0.1)

        # ================================
        # 5 CARTESIAN CONTROL LOOP
        # ================================
        self.get_logger().info("Phase 1: Cartesian Control")

        motion = MotionUpdate()
        motion.header.frame_id = "base_link"

        motion.pose = Pose(
            position=Point(
                x=cartesian_target3[0],
                y=cartesian_target3[1],
                z=cartesian_target3[2],
            ),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
        )

        motion.target_stiffness = np.diag([85.0]*6).flatten()
        motion.target_damping = np.diag([75.0]*6).flatten()
        motion.feedforward_wrench_at_tip = Wrench()
        motion.wrench_feedback_gains_at_tip = [0.0]*6

        motion.trajectory_generation_mode.mode = (
            TrajectoryGenerationMode.MODE_POSITION
        )

        while True:
            obs = get_observation()
            if obs is None:
                continue

            tcp = obs.controller_state.tcp_pose.position

            current_pos = np.array([tcp.x, tcp.y, tcp.z])
            error = np.linalg.norm(current_pos - cartesian_target3)

            self.get_logger().info(f"[Cartesian] Error: {error:.4f}")

            if error < cartesian_threshold:
                self.get_logger().info("Cartesian target reached ✅")
                break

            motion.header.stamp = self.get_clock().now().to_msg()
            move_robot(motion_update=motion)

            send_feedback(f"Cartesian moving... error={error:.4f}")
            time.sleep(0.1)
        # ================================
        # ✅ DONE
        # ================================
        self.get_logger().info("=== TASK COMPLETED SUCCESSFULLY ===")
        return True
