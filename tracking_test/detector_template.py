#!/usr/bin/env python3
"""
detector_node.py  – template

• Subscribes to /image_raw (sensor_msgs/Image)
• Runs your detector (YOLO, Faster-RCNN, etc.)
• Optionally hands detections to a tracker (e.g. ByteTrack / DeepSORT)
• Publishes Detection2DArray on /detections

Fill in the sections marked  # TODO  with your own model / tracker code.
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge

# ---------------- ROS msg types ----------------
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D, Detection2DArray,
    ObjectHypothesisWithPose, Pose2D, Point2D
)

# ------------- class / id constants ------------
HUMAN_CLASS_ID = 1            # integer label for “person”
CONF_THRESHOLD  = 0.4         # discard detections below this
NMS_THRESHOLD   = 0.5         # if your model needs NMS
# -----------------------------------------------


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")

        # --- Parameters you may expose via launch ---
        self.declare_parameter("model_path", "weights/yolov8n.onnx")
        self.declare_parameter("conf_threshold", CONF_THRESHOLD)
        self.declare_parameter("nms_threshold", NMS_THRESHOLD)

        self.conf_thres = float(self.get_parameter("conf_threshold").value)
        self.nms_thres  = float(self.get_parameter("nms_threshold").value)
        model_path = self.get_parameter("model_path").value

        # --------------------------------------------
        # TODO 1:  load your detector  ----------------
        # Example using OpenCV DNN (ONNX):
        # self.net = cv2.dnn.readNet(model_path)
        # --------------------------------------------
        self.net = None  # placeholder

        # TODO 2 (optional): initialise your tracker --
        #   e.g. self.tracker = ByteTrack()
        # --------------------------------------------
        self.tracker = None  # placeholder

        self.bridge = CvBridge()

        self.create_subscription(Image, "/image_raw", self.img_cb, 10)
        self.det_pub = self.create_publisher(
            Detection2DArray, "/detections", 10
        )

        self.next_track_id = 100  # for simple ID assignment
        self.get_logger().info("DetectorNode ready.")

    # ------------------------------------------------
    # UTIL: convert xyxy → Pose2D centre + size
    # ------------------------------------------------
    @staticmethod
    def xyxy_to_center_size(x1, y1, x2, y2):
        w = x2 - x1
        h = y2 - y1
        cx = x1 + w / 2.0
        cy = y1 + h / 2.0
        return cx, cy, w, h

    # ------------------------------------------------
    def img_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h_img, w_img = cv_img.shape[:2]

        # --------------------------------------------
        # TODO 3:  run inference, return list of
        #   (class_id, conf, x1, y1, x2, y2)
        #   coordinates in pixel space.
        # Example stub returns empty list.
        # --------------------------------------------
        detections = self.run_model(cv_img)

        # Optional: pass detections through tracker
        if self.tracker:
            detections = self.tracker.update(detections)

        # Pack into Detection2DArray
        det_array = Detection2DArray()
        det_array.header = msg.header

        for det in detections:
            class_id, conf, x1, y1, x2, y2, track_id = det
            if conf < self.conf_thres or class_id != HUMAN_CLASS_ID:
                continue

            cx, cy, w, h = self.xyxy_to_center_size(x1, y1, x2, y2)

            det_msg = Detection2D()
            det_msg.header = msg.header

            center = Pose2D()
            center.position = Point2D()
            center.position.x = float(cx)
            center.position.y = float(cy)
            center.theta = 0.0
            det_msg.bbox.center = center
            det_msg.bbox.size_x = float(w)
            det_msg.bbox.size_y = float(h)

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = HUMAN_CLASS_ID
            hypo.hypothesis.score    = float(conf)
            det_msg.results.append(hypo)

            det_msg.id = str(track_id)
            det_array.detections.append(det_msg)

        self.det_pub.publish(det_array)

    # ------------------------------------------------
    # TODO 3: replace this stub with real inference
    # ------------------------------------------------
    def run_model(self, img_bgr):
        """
        Returns list of tuples:
        (class_id, conf, x1, y1, x2, y2, track_id)

        You have three common scenarios:

        A) Detector already does its own tracking
           (e.g. YOLO-NAS with ByteTrack head)
           → use its track_id directly.

        B) Detector only gives boxes
           → call an external tracker, assign IDs.

        C) No tracker at all
           → just increment self.next_track_id per detection.
        """
        height, width = img_bgr.shape[:2]

        # --------- Example using dummy values ----------
        dummy = []  # empty list means “no detections”
        return dummy

    # ------------------------------------------------
    # If you need a fallback simple tracker (IOU based)
    # just to hand out IDs, here is a tiny stub:
    # ------------------------------------------------
    def _simple_id(self):
        self.next_track_id += 1
        return self.next_track_id - 1


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
