#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cv2
import os
import numpy as np
from cv_bridge import CvBridge

class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/slam/pose',  # Tópico correto para as informações de pose do ORB-SLAM3
            self.pose_callback,
            10)
        self.pose_subscription

        self.map_save_path = os.path.expanduser("~")
        self.map_image = None

    def pose_callback(self, msg):
        if self.map_image is None:
            self.map_image = 255 * np.ones((500, 500, 3), dtype=np.uint8)

        x = int(msg.pose.position.x * 100) + 250
        y = int(msg.pose.position.y * 100) + 250

        cv2.circle(self.map_image, (x, y), 2, (0, 0, 255), -1)

        self.get_logger().info(f'Recebida pose: x={msg.pose.position.x}, y={msg.pose.position.y}')

        if msg.header.stamp.sec % 100 == 0:
            image_filename = os.path.join(self.map_save_path, 'map_image.png')
            cv2.imwrite(image_filename, self.map_image)
            self.get_logger().info(f'Imagem de mapa salva em {image_filename}')

            # Salve um arquivo CSV contendo as informações de pose
            csv_filename = os.path.join(self.map_save_path, 'pose_info.csv')
            with open(csv_filename, 'a') as csv_file:
                csv_file.write(f'Stamp,Position_X,Position_Y\n')
                csv_file.write(f'{msg.header.stamp.sec},{msg.pose.position.x},{msg.pose.position.y}\n')
            self.get_logger().info(f'Dados de pose salvos em {csv_filename}')

def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
