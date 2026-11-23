import rclpy
from rclpy.node import Node

import time

import cv2

from nectar.ai.detection import Detector
from sensor_msgs.msg import CompressedImage

from faulty_or_not.parameters import (
    DEVICE
)

import os

from dshw.constants import(
    LOOP_HZ,
    DEVICE,
)

MAX_DURATION_S = 15.0

class GaugeReading(Node):

    def __init__(self):
        super().__init__('gauge_reading')

        self.confidence_threshold = 0.5

        self.get_logger().info(f"[GaugeReading] Loading model...")

        self.detector = self._load_model("/home/jetson/ros2_ws/src/dshw/dshw/models/best.pt", "classification")
        self.inference_image_publisher = self.create_publisher(CompressedImage, "gauge/reading/compressed", 10)

        self.get_logger().info(f"[GaugeReading] Model loaded succesfully.")

        self.cam = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
        time.sleep(1)
        if not self.cam.isOpened():
            self.get_logger().error(f"[GaugeReading] Failed to open camera device: {DEVICE}")

        timer_period = 1.0 / LOOP_HZ
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.frame_count = 0
        
        self.get_logger().info(f"[GaugeReading] Control loop running at {LOOP_HZ} Hz")
        self.get_logger().info("[GaugeReading] Starting classification...")

    def _load_model(self, path: str, label: str):
        try:
            m = Detector(model_source=path, framework="ultralytics", confidence_threshold=self.confidence_threshold)
            m.load()
            return m
        except Exception as e:
            self.get_logger().error(f"Error loading {label} model: {e}")
            return None

    def _read_frame(self):
        return self.cam.read()

    def _publish_frame(self, frame):
        save_dir = os.path.join(os.path.expanduser("~"), "faulty_inferences")
        os.makedirs(save_dir, exist_ok=True)

        _, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = buffer.tobytes()
        self.inference_image_publisher.publish(msg)

    def _get_square_crops(self, frame, overlap=0.3):
        h, w = frame.shape[:2]
        size = min(h, w)
        step = int(size * (1 - overlap))
        crops = []
        if w >= h:
            x = 0
            while x + size <= w:
                crops.append((frame[0:size, x:x + size], x, 0))
                x += step
            if not crops or crops[-1][1] + size < w:
                crops.append((frame[0:size, w - size:w], w - size, 0))
        else:
            y = 0
            while y + size <= h:
                crops.append((frame[y:y + size, 0:size], 0, y))
                y += step
            if not crops or crops[-1][2] + size < h:
                crops.append((frame[h - size:h, 0:size], 0, h - size))
        return crops

    def _best_detection_in_frame(self, frame, detector, threshold):
        best_det, best_conf, best_x_off, best_y_off, best_ann = None, 0.0, 0, 0, None
        for crop, x_off, y_off in self._get_square_crops(frame):
            result = detector.detect(crop, conf=threshold)
            detections = result if isinstance(result, list) else result.detections
            for det in detections:
                if det.confidence > best_conf:
                    best_det, best_conf = det, det.confidence
                    best_x_off, best_y_off = x_off, y_off
                    best_ann = detector.draw_detections(crop, result)
        return best_det, best_x_off, best_y_off, best_ann

    def control_loop(self):

        ok, frame = self._read_frame()
        if not ok or frame is None:
            self.get_logger().warn("[GaugeReading] Failed to capture frame. Skipping loop.")
            return
        self.frame_count += 1

        det, _, _, ann = self._best_detection_in_frame(
            frame, self.detector, self.confidence_threshold
        )

        frame = self._get_square_crops(frame)

        if det is None:
            self.get_logger().info("[GaugeReading] No detection found.")
        
        else:

            text = f"{det.confidence * 100.0}% {det.class_id}"
            coordinates = (50, 100)
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            color = (0, 0, 0)
            thickness = 2

            cv2.putText(ann, text, coordinates, font, font_scale, color, thickness)

            class_id = det.class_id
            conf = det.confidence
            self.get_logger().info(
                f"[GaugeReading] Frame {self.frame_count}: class={class_id} "
                f"conf={conf:.3f}"
            )

            save_path = os.path.expanduser(f"~/ros2_ws/src/saved_images/dshw/{time.time()}.jpg")
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            cv2.imwrite(save_path, ann)

        self._publish_frame(ann if det is not None else frame)

        self.get_logger().info(
            f"[DATA_LOG] Classification | Frame {self.frame_count} | "
        )

    def cleanup(self) -> None:
        """Release camera resources."""
        if self._is_shutdown:
            return
        self._is_shutdown = True
        self.image_handler.cleanup()
        self.get_logger().info("Camera publisher stopped")



def main(args=None):
    rclpy.init(args=args)
    node = GaugeReading()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()