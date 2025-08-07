import rclpy
from rclpy.node import Node

import json
import os
import time
import cv2

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Pose2D, Point2D

from cv_bridge import CvBridge

# OLD: class_name = label['Class']
# Use class = "human", id = number parsed from "human1"

import re 

class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')

        # Parameters (update these or declare as launch params)
        self.declare_parameter('json_path', 'labels.json')
        self.declare_parameter('image_dir', './images')
        self.declare_parameter('publish_rate', 5.0)  # Hz

        self.json_path = self.get_parameter('json_path').get_parameter_value().string_value
        self.image_dir = self.get_parameter('image_dir').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.gt_pub = self.create_publisher(Detection2DArray, '/ground_truth', 10)

        self.bridge = CvBridge()

        self.labels = self.load_json(self.json_path)
        self.frame_index = 0

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info('GroundTruthPublisher initialized.')
        self.HUMAN_CLASS_ID = 0  # choose 0 or 1, but keep it fixed

    def load_json(self, json_path):
        with open(json_path, 'r') as f:
            return json.load(f)

    def timer_callback(self):
        if self.frame_index >= len(self.labels):
            self.get_logger().info('All frames published. Stopping...')
            rclpy.shutdown() 
            # self.destroy_timer(self.timer)
            return

        frame_data = self.labels[self.frame_index]
        img_path = os.path.join(self.image_dir, frame_data['File'])

        # Load image
        img = cv2.imread(img_path)
        if img is None:
            self.get_logger().warn(f'Could not load image: {img_path}')
            self.frame_index += 1
            return

        # Convert to ROS image message
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish image
        self.image_pub.publish(img_msg)

        # Construct Detection2DArray message
        gt_msg = Detection2DArray()
        gt_msg.header = img_msg.header

        for label in frame_data['Labels']:
            det = Detection2D()
            det.header = img_msg.header

            x, y, w, h = label['BoundingBoxes']

            center = Pose2D()
            center.position = Point2D()
            center.position.x = float(x + w / 2.0)
            center.position.y = float(y + h / 2.0)
            center.theta = 0.0

            det.bbox.center = center
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            original_class = label['Class']
            match = re.match(r'(.*?)(\d+)$', original_class)

            if match:
                base_class = match.group(1)
                obj_id = int(match.group(2))
            else:
                base_class = original_class
                obj_id = 0

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = str(self.HUMAN_CLASS_ID)   # object class = human
            hypo.hypothesis.score    = 1.0

            det.results.append(hypo)
            det.id = str(obj_id)
            gt_msg.detections.append(det)


        # Publish ground truth
        self.gt_pub.publish(gt_msg)

        # self.get_logger().info(f'Published frame {self.frame_index + 1}/{len(self.labels)}')
        self.frame_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
