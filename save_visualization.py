#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/bevnet/visualization',
            self.callback,
            1
        )
        self.count = 0
        
    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            filename = f'bevnet_viz_{self.count:04d}.png'
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'Saved {filename}')
            self.count += 1
            
            # 5张后退出
            if self.count >= 5:
                self.get_logger().info('Saved 5 images, exiting...')
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main():
    rclpy.init()
    node = ImageSaver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
