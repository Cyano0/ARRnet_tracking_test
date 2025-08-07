#!/usr/bin/env python3
"""
mock_detection_publisher.py

Subscribes to the ground-truth Detection2DArray, adds configurable noise /
dropouts / ID re-mapping, and republishes a fake detector output.
"""

import rclpy
from rclpy.node import Node
import random

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from vision_msgs.msg import Pose2D, Point2D
from sensor_msgs.msg import Image  # only for header stamping

# ---------- tweakable defaults ----------
HUMAN_CLASS_ID = 1          # must match your GT choice
ID_START       = 100        # detection IDs will be 100,101,…
POS_JITTER_PX  = 5.0        # ± pixels added to centre
SIZE_JITTER_PCT = 0.10      # ±10 % on width/height
DROP_PROB      = 0.05       # 5 % chance to drop a box completely
ID_SWITCH_PROB = 0.05       # 5 % chance to swap two IDs in a frame
# ----------------------------------------

class MockDetectionPublisher(Node):
    def __init__(self):
        super().__init__("mock_detection_publisher")

        self.declare_parameter("pos_jitter_px", POS_JITTER_PX)
        self.declare_parameter("size_jitter_pct", SIZE_JITTER_PCT)
        self.declare_parameter("drop_prob", DROP_PROB)
        self.declare_parameter("id_switch_prob", ID_SWITCH_PROB)

        self.pos_jitter_px  = float(self.get_parameter("pos_jitter_px").value)
        self.size_jitter_pct = float(self.get_parameter("size_jitter_pct").value)
        self.drop_prob       = float(self.get_parameter("drop_prob").value)
        self.id_switch_prob  = float(self.get_parameter("id_switch_prob").value)

        # map GT track-id ➜ detection track-id
        self.id_map = {}
        self.next_det_id = ID_START

        self.det_pub = self.create_publisher(
            Detection2DArray, "/detections", 10
        )
        self.gt_sub = self.create_subscription(
            Detection2DArray, "/ground_truth", self.gt_cb, 10
        )

        self.get_logger().info("MockDetectionPublisher ready.")

    # ------------- helpers -------------
    def _gt2det_id(self, gt_id: int) -> int:
        """Stable mapping GT-id → detection-id."""
        if gt_id not in self.id_map:
            self.id_map[gt_id] = self.next_det_id
            self.next_det_id += 1
        return self.id_map[gt_id]
    # -----------------------------------

    def gt_cb(self, gt_msg: Detection2DArray):
        det_array = Detection2DArray()
        det_array.header = gt_msg.header

        detections_this_frame = []

        for gt_det in gt_msg.detections:
            # maybe drop this detection
            if random.random() < self.drop_prob:
                continue

            # pull GT centre / size
            cx = gt_det.bbox.center.position.x
            cy = gt_det.bbox.center.position.y
            w  = gt_det.bbox.size_x
            h  = gt_det.bbox.size_y

            # add Gaussian noise
            cx += random.gauss(0.0, self.pos_jitter_px)
            cy += random.gauss(0.0, self.pos_jitter_px)
            w  *= 1.0 + random.gauss(0.0, self.size_jitter_pct)
            h  *= 1.0 + random.gauss(0.0, self.size_jitter_pct)

            # build detection message
            det = Detection2D()
            det.header = gt_msg.header

            center = Pose2D()
            center.position = Point2D()
            center.position.x = float(cx)
            center.position.y = float(cy)
            center.theta = 0.0
            det.bbox.center = center
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            # map GT id to a new detection id
            gt_track_id = int(gt_det.id) if gt_det.id else 0
            det_track_id = self._gt2det_id(gt_track_id)
            det.id = str(det_track_id)                     # <-- tracker ID

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = str(HUMAN_CLASS_ID)  # always human
            hypo.hypothesis.score    = 0.9             # arbitrary conf.
            det.results.append(hypo)

            detections_this_frame.append(det)

        # optional ID-switch chaos
        if len(detections_this_frame) >= 2 and random.random() < self.id_switch_prob:
            a, b = random.sample(detections_this_frame, 2)
            a.id, b.id = b.id, a.id  # swap IDs

        det_array.detections = detections_this_frame
        self.det_pub.publish(det_array)


def main(args=None):
    rclpy.init(args=args)
    node = MockDetectionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
