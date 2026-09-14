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

class TestPathV1(Policy):
    
    # ---------------------------------------------------------
    # NESTED PATH CLASS (Trajectory Logic)
    # ---------------------------------------------------------
    class Path:
        class Subdivision:
            def __init__(self, A, B, C, radius=1.0, segments=3):
                self.A = np.array(A, dtype=float)
                self.B = np.array(B, dtype=float)
                self.C = np.array(C, dtype=float)
                self.r = radius
                self.segments = segments
                
            def normalize(self, v):
                norm = np.linalg.norm(v)
                if norm == 0:
                    raise ValueError("Zero-length vector")
                return v / norm

            def angle_between(self, v1, v2):
                dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
                return np.arccos(dot)

            def compute(self):
                dist_AB = np.linalg.norm(self.A - self.B)
                dist_CB = np.linalg.norm(self.C - self.B)
                max_allowed_r = min(dist_AB, dist_CB) / 2.0
                safe_r = min(self.r, max_allowed_r)

                dir1 = self.normalize(self.A - self.B)
                dir2 = self.normalize(self.C - self.B)

                start = self.B + dir1 * safe_r
                end   = self.B + dir2 * safe_r
                angle = self.angle_between(dir1, dir2)

                arc_points = []

                if angle > (np.pi - 1e-6):
                    for i in range(1, self.segments + 1):
                        t = i / (self.segments + 1)
                        p = start + t * (end - start)
                        arc_points.append(p)
                    return {"start": start, "arc_points": arc_points, "end": end, "center": self.B}
                
                if angle < 1e-6:
                    raise ValueError("Points fold back perfectly. Cannot fillet.")

                bisector = self.normalize(dir1 + dir2)
                dist = safe_r / np.sin(angle / 2.0)
                center = self.B + bisector * dist

                start_vec = start - center
                end_vec   = end - center
                normal = self.normalize(np.cross(start_vec, end_vec))
                total_angle = self.angle_between(start_vec, end_vec)

                for i in range(1, self.segments + 1):
                    t = i / (self.segments + 1)
                    theta = total_angle * t
                    v = start_vec
                    k = normal

                    v_rot = (v * np.cos(theta) +
                         np.cross(k, v) * np.sin(theta) +
                         k * np.dot(k, v) * (1 - np.cos(theta)))

                    arc_radius = np.linalg.norm(start_vec)
                    p = center + self.normalize(v_rot) * arc_radius
                    arc_points.append(p)

                return {"start": start, "arc_points": arc_points, "end": end, "center": center}

        def __init__(self):
            self.sequence = []
            
        def add_order(self, *args):
            if len(args) == 3:
                self.sequence.append([float(args[0]), float(args[1]), float(args[2])])
            elif len(args) == 1:
                msg = args[0]
                if hasattr(msg, 'position'):
                    self.sequence.append([msg.position.x, msg.position.y, msg.position.z])
                elif hasattr(msg, 'x'):
                    self.sequence.append([msg.x, msg.y, msg.z])
                else:
                    raise ValueError("Unsupported message type.")
            else:
                raise ValueError("Requires (x, y, z) or single geometry_msgs object.")
        
        def current_order(self):
            if self.sequence:    # check not empty
                return self.sequence[0]
            return None
                
        def order_completed(self):
            if self.sequence:    # check not empty
                self.sequence.pop(0)
            else:
                print("Sequence is empty")
            
        def apply_fillet(self, radius=1.0, segments=3):
            if len(self.sequence) < 3:
                return self.sequence.copy()

            new_path = [self.sequence[0]]

            for i in range(1, len(self.sequence) - 1):
                A = self.sequence[i - 1]
                B = self.sequence[i]
                C = self.sequence[i + 1]

                fillet = self.Subdivision(A, B, C, radius, segments)
                try:
                    result = fillet.compute()
                    new_path.append(result["start"].tolist())
                    for p in result["arc_points"]:
                        new_path.append(p.tolist())
                    new_path.append(result["end"].tolist())
                except ValueError:
                    new_path.append(B)

            new_path.append(self.sequence[-1])
            return new_path

    # ---------------------------------------------------------
    # POLICY INTEGRATION
    # ---------------------------------------------------------
    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("TestPathV1 initialized")

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        **kwargs,
    ):
        self.get_logger().info(f"Executing smoothed sequence A-B-C-D-E. Task: {task}")
        send_feedback("Calculating smoothed trajectory...")

        # 1. Define the fixed points (A, B, C, D, E)
        point_a = [0, 0.5, 0]
        point_b = [0.5, 0.5, 0]
        point_c = [0.5, 1, 0]
        point_d = [-0.5, 0, 0]
        point_e = [0, 0.5, 0]
        # 2. Load them into the Path manager
        path_manager = self.Path()
        path_manager.add_order(*point_a)
        path_manager.add_order(*point_b)
        path_manager.add_order(*point_c)
        path_manager.add_order(*point_d)
        path_manager.add_order(*point_e)

        # 3. Smooth the corners of this fixed path with a fillet
        # We REASSIGN this back to path_manager.sequence so the queue methods work!
        path_manager.sequence = path_manager.apply_fillet(radius=0.05, segments=5)
        
        total_waypoints = len(path_manager.sequence)
        send_feedback(f"Moving through {total_waypoints} generated waypoints...")

        # 4. Command the robot through the sequence using the queue manager
        idx = 0
        while path_manager.current_order() is not None:
            # Grab the point from the front of the queue
            point = path_manager.current_order()
            x, y, z = point
            
            # Fetch observation (optional, ensures state is fresh)
            get_observation()

            self.get_logger().info(f"Moving to Waypoint {idx}/{total_waypoints-1}: x={x:.3f}, y={y:.3f}, z={z:.3f}")

            # 🎯 Target pose for this waypoint
            pose = Pose(
                position=Point(x=x, y=y, z=z),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
            )

            # Construct MotionUpdate
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

            # Trajectory mode
            motion_update.trajectory_generation_mode.mode = (
                TrajectoryGenerationMode.MODE_POSITION
            )

            # 🚀 Send move command
            move_robot(motion_update=motion_update)

            # ---------------------------------------------------------
            # WAIT FOR FEEDBACK (Arrival at target)
            # Currently using sleep, but you could replace this with a 
            # while loop that checks `get_observation()` distance to target.
            time.sleep(1.5) 
            # ---------------------------------------------------------

            # Robot has reached the point (feedback received). Pop it from the queue!
            path_manager.order_completed() 
            idx += 1

        self.get_logger().info("Sequence queue is empty. Robot stopped.")
        send_feedback("Fixed sequence completed. Robot stopped.")
        
        return True
