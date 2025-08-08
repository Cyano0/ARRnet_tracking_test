#!/usr/bin/env python3
"""
Draws ground-truth (green) and detections (blue for TP, red for FP)
on the live image stream and republishes as /eval_viz.
"""

import rclpy, time
from rclpy.node import Node
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

# colour BGR
GT_COLOR       = (  0,255,  0)   # green
DET_TP_COLOR   = (255,  0,  0)   # blue
DET_FP_COLOR   = (  0,  0,255)   # red
FONT           = cv2.FONT_HERSHEY_SIMPLEX

IOU_TH = 0.5                      # duplicate of evaluator’s threshold

def box_to_xyxy(det):
    cx = det.bbox.center.position.x
    cy = det.bbox.center.position.y
    w  = det.bbox.size_x
    h  = det.bbox.size_y
    return np.array([cx-w/2, cy-h/2, cx+w/2, cy+h/2], dtype=np.float32)

def iou(a,b):
    inter = max(0,min(a[2],b[2])-max(a[0],b[0])) * \
            max(0,min(a[3],b[3])-max(a[1],b[1]))
    if inter == 0: return 0.0
    area_a = (a[2]-a[0])*(a[3]-a[1])
    area_b = (b[2]-b[0])*(b[3]-b[1])
    return inter/(area_a+area_b-inter)

class Visualiser(Node):
    def __init__(self):
        super().__init__("visualiser")
        self.get_logger().info("Visualiser ready.")
        self.bridge = CvBridge()

        img_sub  = message_filters.Subscriber(self, Image, "/image_raw")
        gt_sub   = message_filters.Subscriber(self, Detection2DArray, "/ground_truth")
        det_sub  = message_filters.Subscriber(self, Detection2DArray, "/detections")

        sync = message_filters.ApproximateTimeSynchronizer(
                [img_sub, gt_sub, det_sub], queue_size=20, slop=0.05)
        sync.registerCallback(self.cb)

        self.pub = self.create_publisher(Image, "/eval_viz", 10)
        self.get_logger().info("Visualiser ready (view /eval_viz in rqt_image_view)")

    def cb(self, img_msg, gt_msg, det_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg).copy()

        gt_boxes = [box_to_xyxy(d) for d in gt_msg.detections]
        det_boxes = [box_to_xyxy(d) for d in det_msg.detections]

        matched_gt = set()

        # draw ground-truth
        for i, box in enumerate(gt_boxes):
            x1,y1,x2,y2 = map(int, box)
            cv2.rectangle(frame,(x1,y1),(x2,y2),GT_COLOR,2)
            cv2.putText(frame, f'GT {gt_msg.detections[i].id}',
                        (x1,y1-5), FONT, 0.5, GT_COLOR, 1)

        # draw detections
        for di, dbox in enumerate(det_boxes):
            best_iou = 0
            for gi, gbox in enumerate(gt_boxes):
                if gi in matched_gt: continue
                i = iou(dbox,gbox)
                if i>best_iou:
                    best_iou, best_gi = i, gi
            color = DET_TP_COLOR if best_iou>=IOU_TH else DET_FP_COLOR
            if best_iou>=IOU_TH: matched_gt.add(best_gi)

            x1,y1,x2,y2 = map(int, dbox)
            cv2.rectangle(frame,(x1,y1),(x2,y2),color,2)
            cv2.putText(frame, f'ID {det_msg.detections[di].id}',
                        (x1,y2+15), FONT, 0.5, color, 1)

        # publish
        out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = img_msg.header
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Visualiser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
