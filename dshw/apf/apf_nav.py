import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped
import numpy as np

from nectar.control import (
    MavrosDrone,
    DroneFactory,
    MoveReference,
    MavrosConfig,
    SITL_VISION_CONFIG,
)

from nectar.control.types import PoseSource

from dshw.constants import (
    GOAL_M,
    K_ATT,
    K_REP,
    V_MAX,
    GOAL_TOL,
    TIMEOUT,
    LOOP_HZ,
    OBSTACLE_NS,
)

from .apf_math import attractive_force, repulsive_force


class ApfNavigationNode(Node):
    def __init__(self):
        super().__init__('apf_navigation_node')
        
        self.declare_parameter('mode', 'real') # real, cam, gz
        self.test = self.get_parameter('mode').value

        if self.test == 'real':
            self.get_logger().info("Initializing drone...")

            self.config = MavrosConfig(
                pose_source=PoseSource.VISION,
            )

            self.drone : MavrosDrone = DroneFactory.create('mavros', self.config)

        elif self.test == 'gz':
            self.get_logger().info("Initializing drone simulation...")
            
            self.config = SITL_VISION_CONFIG

            self.drone = DroneFactory.create('mavros', self.config)

        elif self.test == 'cam':
            self.get_logger().info("Camera-only test mode initialized.")
            
        
        self.latest_bboxes = []

        bbox_topic = f"/{OBSTACLE_NS}/obstacle_bboxes"
        self.bbox_sub = self.create_subscription(
            Float32MultiArray,
            bbox_topic,
            self.bbox_callback,
            10
        )

        self.is_obstacle_detected = False

        # 2. Setup the new subscriber
        detected_topic = f"/{OBSTACLE_NS}/obstacle_detected"
        self.detected_sub = self.create_subscription(
            Bool,
            detected_topic,
            self.detected_callback,
            10
        )

        self.get_logger().info(
            f"[ApfNavigationNode] Initialized. goal={GOAL_M}, k_att={K_ATT:.2f}, "
            f"k_rep={K_REP:.2f}, v_max={V_MAX:.2f}"
        )

        self.start_time = self.get_clock().now()
        
        timer_period = 1.0 / LOOP_HZ
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info(f"[ApfNavigationNode] Control loop running at {LOOP_HZ} Hz")

    # ── Callbacks ─────────────────────────────────────────────────

    def bbox_callback(self, msg: Float32MultiArray):
        self.latest_bboxes = list(msg.data)
    
    def detected_callback(self, msg: Bool):
        self.is_obstacle_detected = msg.data

    # ── Control Loop ──────────────────────────────────────────────
    def control_loop(self):
        
        now = self.get_clock().now()
        elapsed_seconds = (now - self.start_time).nanoseconds / 1e9
        
        if elapsed_seconds > TIMEOUT:
            self.get_logger().error(f"[ApfNavigationNode] Timeout reached after {TIMEOUT:.1f} s. Aborting!")
            self.stop_drone_and_shutdown()
            return

        raw_pos = self.drone.position
        if raw_pos is None:
            return 
            
        if isinstance(raw_pos, PoseStamped):
            pos = np.array([
                raw_pos.pose.position.x,
                raw_pos.pose.position.y,
                raw_pos.pose.position.z
            ])
        else:
            self.get_logger().warning("Received coordinate type that is not a local PoseStamped!")
            return

        if self.is_obstacle_detected:
            bboxes = list(self.latest_bboxes)
        else:
            bboxes = []

        dist_to_goal = np.linalg.norm(GOAL_M - pos)
        if dist_to_goal < GOAL_TOL:
            self.get_logger().info(f"[ApfNavigationNode] Goal reached! final distance={dist_to_goal:.3f} m")
            self.stop_drone_and_shutdown()
            return

        f_att = attractive_force(pos)
        f_rep = repulsive_force(bboxes) 
        f_total = f_att + f_rep

        v_norm = np.linalg.norm(f_total)
        if v_norm > V_MAX:
            f_total = (f_total / v_norm) * V_MAX

        if self.test != 'cam': 
            try:
                self.drone.move_velocity(
                    f_total[0], 
                    f_total[1], 
                    0.0, 
                    0.0, 
                    reference=MoveReference.BODY
                )
            except Exception as e:
                self.get_logger().error(f"[ApfNavigationNode] Flight command failed: {e}")
                self.stop_drone_and_shutdown()
        
        num_clusters = len(bboxes) // 5
        self.get_logger().info(
            f"APF Output -> vx: {f_total[0]:.3f}, vy: {f_total[1]:.3f} | "
            f"Obs Count: {num_clusters} | Dist to Goal: {dist_to_goal:.2f}m | "
        )

    def stop_drone_and_shutdown(self):
        """Safely zeros drone velocities and cancels the loop timer."""
        self.get_logger().info("[ApfNavigationNode] Sending safe hover velocity...")
        try:
            self.drone.move_velocity(0.0, 0.0, 0.0, 0.0, reference=MoveReference.BODY)
        except Exception as e:
            self.get_logger().error(f"Could not issue zero-velocity fallback command: {e}")
        
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = ApfNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt captured. Exiting cleanly...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()