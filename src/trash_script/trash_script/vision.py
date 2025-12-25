#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # --- ROS pub/sub ---
        self.pub_target = self.create_publisher(Point, '/target_coord', 10)
        self.pub_dbgimg = self.create_publisher(Image, '/yolo/debug_image', 10)
        self.pub_state  = self.create_publisher(String, '/vision/state', 10)

        self.sub_img = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.bridge = CvBridge()
        self.latest_frame = None

        # --- Parameters (可不改也能跑) ---
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('yolo_every_n', 1)   # 1=每帧跑YOLO；2=每2帧；3=每3帧...
        self.declare_parameter('use_tracker', True)

        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.yolo_every_n = int(self.get_parameter('yolo_every_n').value)
        self.use_tracker = bool(self.get_parameter('use_tracker').value)

        # --- Load model (從 share 取 best.pt) ---
        pkg_share = get_package_share_directory('trash_script')
        self.model_path = os.path.join(pkg_share, 'best2.pt')
        self.get_logger().info(f"Using model: {self.model_path}")

        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info("YOLO loaded OK.")
        except Exception as e:
            self.get_logger().error(f"Failed to load best.pt: {e}. Fallback to yolov8n.pt")
            self.model = YOLO('yolov8n.pt')

        # --- State machine ---
        self.state = "SEARCH"
        self.tracker = None
        self.track_bbox = None  # (x, y, w, h)

        # --- counters / throttles ---
        self.frame_count = 0
        self.last_print_t = 0.0

        # main loop
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30Hz

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def timer_callback(self):
        if self.latest_frame is None:
            return

        frame = self.latest_frame
        h, w, _ = frame.shape
        t = self.get_clock().now().nanoseconds / 1e9

        # publish state (低頻)
        if t - self.last_print_t > 0.5:
            s = String()
            s.data = self.state
            self.pub_state.publish(s)

        debug_frame = frame.copy()
        target_msg = None

        if self.state == "TRACK" and self.use_tracker and self.tracker is not None:
            ok, bbox = self.tracker.update(frame)
            if ok:
                x, y, bw, bh = [int(v) for v in bbox]
                x = max(0, min(x, w - 1))
                y = max(0, min(y, h - 1))
                bw = max(1, min(bw, w - x))
                bh = max(1, min(bh, h - y))
                self.track_bbox = (x, y, bw, bh)

                cx = x + bw / 2.0
                cy = y + bh / 2.0
                norm_x = (cx - w/2) / (w/2)
                norm_y = -(cy - h/2) / (h/2)
                norm_area = (bw * bh) / float(w * h)

                target_msg = Point()
                target_msg.x = float(norm_x)
                target_msg.y = float(norm_y)
                target_msg.z = float(norm_area)

                # draw bbox
                cv2.rectangle(debug_frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                cv2.putText(debug_frame, "TRACK", (x, max(15, y - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            else:
                # tracker fail -> back to search
                self.state = "SEARCH"
                self.tracker = None
                self.track_bbox = None

        # SEARCH / or TRACK but want relock with YOLO
        self.frame_count += 1
        do_yolo = (self.frame_count % max(1, self.yolo_every_n) == 0)
        if (self.state == "SEARCH") and do_yolo:
            best = self.run_yolo_pick_best(frame)
            if best is not None:
                x1, y1, x2, y2, conf = best
                bw = max(1, x2 - x1)
                bh = max(1, y2 - y1)

                # publish from YOLO (當下就能輸出給 trajectory)
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                norm_x = (cx - w/2) / (w/2)
                norm_y = -(cy - h/2) / (h/2)
                norm_area = (bw * bh) / float(w * h)

                target_msg = Point()
                target_msg.x = float(norm_x)
                target_msg.y = float(norm_y)
                target_msg.z = float(norm_area)

                # draw YOLO bbox
                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(debug_frame, f"YOLO {conf:.2f}", (x1, max(15, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # init tracker
                if self.use_tracker:
                    self.tracker = cv2.TrackerKCF_create()
                    self.tracker.init(frame, (x1, y1, bw, bh))
                    self.track_bbox = (x1, y1, bw, bh)
                    self.state = "TRACK"

        # publish target if any
        if target_msg is not None:
            self.pub_target.publish(target_msg)

            if t - self.last_print_t > 0.5:
                self.get_logger().info(
                    f"[{self.state}] x={target_msg.x:+.3f} y={target_msg.y:+.3f} area(z)={target_msg.z:.4f}"
                )
                self.last_print_t = t
        else:
            # still publish debug prints occasionally
            if t - self.last_print_t > 1.0:
                self.get_logger().info(f"[{self.state}] no target")
                self.last_print_t = t

        # publish debug image
        try:
            out = self.bridge.cv2_to_imgmsg(debug_frame, "bgr8")
            self.pub_dbgimg.publish(out)
        except Exception:
            pass

    def run_yolo_pick_best(self, frame):
        """
        return (x1,y1,x2,y2,conf) or None
        """
        try:
            results = self.model(frame, imgsz=self.imgsz, verbose=False)
        except Exception as e:
            self.get_logger().error(f"YOLO inference error: {e}")
            return None

        best = None
        best_conf = -1.0
        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf >= self.conf_threshold and conf > best_conf:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    best = (x1, y1, x2, y2, conf)
                    best_conf = conf
        return best


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
