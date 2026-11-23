import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np

from nectar.control import MavrosDrone, DroneFactory, MoveReference, MavrosConfig, SITL_VISION_CONFIG
from nectar.control.types import PoseSource

from dshw.constants import GOAL_M, V_MAX, GOAL_TOL, TIMEOUT, LOOP_HZ, OBSTACLE_NS
from .dwa_math import calculate_dwa_velocity

class DwaNavigationNode(Node):
    def __init__(self):
        super().__init__('dwa_navigation_node')
        
        self.declare_parameter('mode', 'real') 
        self.test = self.get_parameter('mode').value

        if self.test == 'real':
            self.get_logger().info("Initializing DWA on real hardware...")
            self.config = MavrosConfig(pose_source=PoseSource.VISION)
            self.drone : MavrosDrone = DroneFactory.create('mavros', self.config)

        elif self.test == 'gz':
            self.get_logger().info("Initializing DWA in simulation...")
            self.config = SITL_VISION_CONFIG
            self.drone = DroneFactory.create('mavros', self.config)

        elif self.test == 'cam':
            self.get_logger().info("DWA Camera-only test mode initialized.")
            
        self.latest_bboxes = []

        bbox_topic = f"/{OBSTACLE_NS}/obstacle_bboxes"
        self.bbox_sub = self.create_subscription(
            Float32MultiArray, bbox_topic, self.bbox_callback, 10
        )

        self.get_logger().info(f"[DwaNode] Ready. Goal={GOAL_M}, v_max={V_MAX:.2f}")

        self.start_time = self.get_clock().now()
        timer_period = 1.0 / LOOP_HZ
        self.timer = self.create_timer(timer_period, self.control_loop)

    def bbox_callback(self, msg: Float32MultiArray):
        self.latest_bboxes = list(msg.data)

    def control_loop(self):
        now = self.get_clock().now()
        elapsed_seconds = (now - self.start_time).nanoseconds / 1e9
        
        if elapsed_seconds > TIMEOUT:
            self.get_logger().error(f"[DwaNode] Timeout after {TIMEOUT:.1f} s. Aborting!")
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
            return

        bboxes = list(self.latest_bboxes)

        dist_to_goal = np.linalg.norm(GOAL_M - pos)
        if dist_to_goal < GOAL_TOL:
            self.get_logger().info(f"[DwaNode] Goal reached! dist={dist_to_goal:.3f} m")
            self.stop_drone_and_shutdown()
            return

        # ---- DWA REPLACES APF HERE ----
        f_total = calculate_dwa_velocity(pos, GOAL_M, bboxes, V_MAX)

        if self.test != 'cam': 
            try:
                self.drone.move_velocity(
                    f_total[0], f_total[1], 0.0, 0.0, reference=MoveReference.BODY
                )
            except Exception as e:
                self.get_logger().error(f"[DwaNode] Flight command failed: {e}")
                self.stop_drone_and_shutdown()
        
        num_clusters = len(bboxes) // 5
        self.get_logger().info(
            f"DWA Output -> vx: {f_total[0]:.3f}, vy: {f_total[1]:.3f} | "
            f"Obs Count: {num_clusters} | Dist to Goal: {dist_to_goal:.2f}m"
        )

    def stop_drone_and_shutdown(self):
        self.get_logger().info("[DwaNode] Sending safe hover velocity...")
        try:
            self.drone.move_velocity(0.0, 0.0, 0.0, 0.0, reference=MoveReference.BODY)
        except Exception:
            pass
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = DwaNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()