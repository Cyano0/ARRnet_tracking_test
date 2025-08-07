#!/usr/bin/env python3
"""
evaluator.py

Compute IoU-based detection/tracking metrics between /ground_truth
(vision_msgs/Detection2DArray) and /detections.
"""

import rclpy
from rclpy.node import Node
import message_filters
import numpy as np

from vision_msgs.msg import Detection2DArray

IOU_THRESHOLD = 0.5      # match threshold
PRINT_EVERY   = 50       # frames

def box_to_xyxy(det):
    cx = det.bbox.center.position.x
    cy = det.bbox.center.position.y
    w  = det.bbox.size_x
    h  = det.bbox.size_y
    x1 = cx - w / 2.0
    y1 = cy - h / 2.0
    x2 = cx + w / 2.0
    y2 = cy + h / 2.0
    return np.array([x1, y1, x2, y2], dtype=np.float32)

def iou(boxA, boxB):
    # box = [x1 y1 x2 y2]
    inter = np.maximum(0, np.minimum(boxA[2], boxB[2]) -
                          np.maximum(boxA[0], boxB[0])) * \
            np.maximum(0, np.minimum(boxA[3], boxB[3]) -
                          np.maximum(boxA[1], boxB[1]))
    if inter == 0:
        return 0.0
    areaA = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    areaB = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    return inter / (areaA + areaB - inter)

class Evaluator(Node):
    def __init__(self):
        super().__init__("evaluator")

        self.tp = 0
        self.fp = 0
        self.fn = 0
        self.id_switches = 0
        self.frame_count = 0
        self.gt2det_prev = {}   # gt-id ➔ det-id

        gt_sub  = message_filters.Subscriber(self, Detection2DArray, "/ground_truth")
        det_sub = message_filters.Subscriber(self, Detection2DArray, "/detections")

        sync = message_filters.ApproximateTimeSynchronizer(
            [gt_sub, det_sub], queue_size=20, slop=0.05
        )
        sync.registerCallback(self.sync_cb)

        self.get_logger().info("Evaluator ready.")
        self.last_msg_time = self.get_clock().now()
        self.timer = self.create_timer(1.0, self.watchdog_cb)

    def watchdog_cb(self):
        if (self.get_clock().now() - self.last_msg_time).nanoseconds > 10e9:
            self.get_logger().info("No ground-truth for 10s, finishing evaluation.")
            self.print_metrics(final=True) 
            rclpy.shutdown()
    # ------- main callback -------
    def sync_cb(self, gt_msg: Detection2DArray, det_msg: Detection2DArray):
        self.last_msg_time = self.get_clock().now()
        self.frame_count += 1

        gt_boxes  = [box_to_xyxy(d) for d in gt_msg.detections]
        gt_ids    = [int(d.id) if d.id else -1 for d in gt_msg.detections]

        det_boxes = [box_to_xyxy(d) for d in det_msg.detections]
        det_ids   = [int(d.id) if d.id else -1 for d in det_msg.detections]

        matched_gt = set()
        matched_det = set()

        # Greedy match (could use Hungarian, but good enough for demo)
        for di, dbox in enumerate(det_boxes):
            best_iou = 0.0
            best_gi  = -1
            for gi, gbox in enumerate(gt_boxes):
                if gi in matched_gt:
                    continue
                i = iou(dbox, gbox)
                if i > best_iou:
                    best_iou = i
                    best_gi  = gi
            if best_iou >= IOU_THRESHOLD:
                matched_det.add(di)
                matched_gt.add(best_gi)
                self.tp += 1

                # --- tracking consistency ---
                gt_id  = gt_ids[best_gi]
                det_id = det_ids[di]
                prev_det = self.gt2det_prev.get(gt_id, det_id)
                if prev_det != det_id:
                    self.id_switches += 1
                self.gt2det_prev[gt_id] = det_id
            else:
                self.fp += 1  # unmatched detection

        # any GT not matched = FN
        self.fn += len(gt_boxes) - len(matched_gt)

        if self.frame_count % PRINT_EVERY == 0:
            self.print_metrics()

        # NEW – always print when publisher is done
        if not gt_msg.detections and not det_msg.detections:
            self.print_metrics()

    # ------- helpers -------
    def precision(self):
        return self.tp / (self.tp + self.fp) if (self.tp + self.fp) else 0.0

    def recall(self):
        return self.tp / (self.tp + self.fn) if (self.tp + self.fn) else 0.0

    def print_metrics(self, final: bool = False):
        tag = "FINAL" if final else f"After {self.frame_count} frames"
        self.get_logger().info(
            f"{tag}:  "
            f"TP={self.tp}  FP={self.fp}  FN={self.fn}  "
            f"P={self.precision():.3f}  R={self.recall():.3f}  "
            f"ID-switches={self.id_switches}"
        )

    # ------- shutdown summary -------
    def destroy_node(self):
        # self.print_metrics()
        self.print_metrics(final=True) 
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_metrics(final=True)
    finally:
        node.destroy_node()
