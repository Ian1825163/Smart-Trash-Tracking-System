import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # --- Configuration ---
        self.model_path = "best2.pt"  # Your trained model file
        self.conf_threshold = 0.5    # Confidence threshold
        # ---------------------

        # 1. Load YOLO Model
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f"Successfully loaded model: {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Model not found! Please check path. Error: {e}")

        # 2. Subscribe to Camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        
        # 3. Publish Control Coordinates
        self.coord_publisher = self.create_publisher(Point, '/trash/coords', 10)

        # 4. Publish Debug Image
        self.img_publisher = self.create_publisher(Image, '/yolo/debug_image', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("YOLO Detector Node Started... (Smart Text Layout)")

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = frame.shape
            
            # YOLO Inference (imgsz=320 for speed)
            results = self.model(frame, imgsz=320, verbose=False)
            
            best_box = None
            best_conf = -1

            for r in results:
                for box in r.boxes:
                    conf = float(box.conf[0])
                    if conf > self.conf_threshold and conf > best_conf:
                        best_conf = conf
                        best_box = box
            
            if best_box is not None:
                x1, y1, x2, y2 = map(int, best_box.xyxy[0])
                
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # Calculations
                norm_x = (center_x - width/2) / (width/2)
                norm_y = -(center_y - height/2) / (height/2)
                box_area = (x2 - x1) * (y2 - y1)
                norm_area = box_area / (width * height)

                # --- Visual Debugging (Draw Box) ---
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # --- Smart Text Positioning 
                # Determine text strings
                label_conf = f"Trash: {best_conf:.2f}"
                label_coords = f"X:{norm_x:.2f} Y:{norm_y:.2f} D:{norm_area:.2f}"
                
                # Check if box is too close to the top edge (less than 40 pixels)
                if y1 < 40:
                    # If too close to top, draw text INSIDE the box
                    # Line 1 (Green)
                    cv2.putText(frame, label_conf, (x1, y1 + 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # Line 2 (Yellow)
                    cv2.putText(frame, label_coords, (x1, y1 + 40), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                else:
                    # Normal case: draw text ABOVE the box
                    # Line 1 (Green)
                    cv2.putText(frame, label_conf, (x1, y1 - 25), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # Line 2 (Yellow)
                    cv2.putText(frame, label_coords, (x1, y1 - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

                # Publish Data
                point_msg = Point()
                point_msg.x = float(norm_x)
                point_msg.y = float(norm_area)
                point_msg.z = float(best_conf)
                self.coord_publisher.publish(point_msg)

            # Publish Debug Image
            out_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.img_publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
