import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import os

class YoloV5Node(Node):
    def __init__(self):
        super().__init__('yolov5_node')
        self.bridge = CvBridge()
        
        script_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(script_dir, 'best.pt')

        self.model = torch.hub.load('ultralytics/yolov5', 'custom',
            path = model_path,
            trust_repo=True)
        self.model.eval()

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(Image, 'yolov5/image_annotated', 10)
        self.signal_pub = self.create_publisher(Bool, 'doll_detected', 10)

        self.detection_count = 0
        self.triggered = False

        self.frame_skip = 3  
        self.frame_count = 0

        self.get_logger().info('YOLOv5 node started')

    def image_callback(self, msg):
        if self.triggered:
            return  # 이미 트리거 되었으면 무시

        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return  # 프레임 스킵

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Image decoding failed: {e}')
            return

        results = self.model(frame, size=320)
        results.render()
        annotated_img = results.ims[0]

        conf_list = results.xyxy[0][:, 4].tolist() if results.xyxy[0].numel() > 0 else []
        detected = any(conf >= 0.7 for conf in conf_list)

        if detected:
            self.detection_count += 1
            self.get_logger().info(f'Detection count: {self.detection_count}')
            if self.detection_count >= 5:
                signal_msg = Bool()
                signal_msg.data = True
                self.signal_pub.publish(signal_msg)
                self.get_logger().info('Target detected! Triggering and shutting down.')
                self.triggered = True
                self.destroy_node()
                rclpy.shutdown()
                return
        else:
            self.detection_count = 0

        try:
            image_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
            image_msg.header.stamp = msg.header.stamp
            image_msg.header.frame_id = 'camera_frame'
            self.publisher.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f'Annotated image publish failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloV5Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


