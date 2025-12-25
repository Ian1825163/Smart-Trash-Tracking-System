#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.pub = self.create_publisher(PointStamped, '/target_coord', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/debug_image', 10)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 10)

        self.bridge = CvBridge()

        # ---- tune ----
        self.min_area = 250
        self.print_dt = 0.30
        self.last_print_t = 0.0
        # ---------------

        self.get_logger().info("Vision HSV-RED started. Put RED tape on ball.")

    def cb(self, img_msg: Image):
        frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        h, w = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # red = two hue ranges
        lower1 = (0,   120, 50)
        upper1 = (10,  255, 255)
        lower2 = (165, 120, 50)
        upper2 = (179, 255, 255)

        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)

        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0.0
        for c in contours:
            a = cv2.contourArea(c)
            if a > best_area:
                best_area = a
                best = c

        if best is not None and best_area >= self.min_area:
            x, y, bw, bh = cv2.boundingRect(best)
            cx = x + bw * 0.5
            cy = y + bh * 0.5

            norm_x = (cx - w / 2) / (w / 2)
            norm_y = -(cy - h / 2) / (h / 2)
            norm_area = (bw * bh) / float(w * h)
            z = 1.0 - norm_area  # 越近 area 越大 -> z 越小

            out = PointStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "camera"
            out.point.x = float(norm_x)
            out.point.y = float(norm_y)
            out.point.z = float(z)
            self.pub.publish(out)

            # debug draw
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.putText(frame, f"x={norm_x:+.2f} y={norm_y:+.2f} z={z:.3f}",
                        (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            t = self.get_clock().now().nanoseconds / 1e9
            if t - self.last_print_t > self.print_dt:
                self.get_logger().info(f"FOUND x={norm_x:+.3f} y={norm_y:+.3f} area={best_area:.0f} z={z:.3f}")
                self.last_print_t = t

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
