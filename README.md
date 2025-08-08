# tracking_test

## Purpose

`tracking_test` is a ROS 2 (Humble) **ament‑python** package that lets you
evaluate people‑detection and multi‑object‑tracking pipelines offline:

* **Publish** a prerecorded image sequence with ground‑truth bounding boxes
  (`ground_truth_publisher.py`).
* **Run** either a *mock* detector (`mock_detection_publisher.py`) or your
  own detector (`detector_node.py` template provided) and publish detections
  on `/detections`.
* **Measure** detection quality (IoU) and ID‑tracking consistency
  (`evaluator.py`).

* **Visualiser** overlays ground truth and detection results on the images for visual inspection.


All communication uses **standard** messages
(`sensor_msgs/Image`, `vision_msgs/Detection2DArray`) – no custom `.msg` files needed.

---

## Directory layout

```
tracking_test/
├── tracking_test/                     # Python module
│   ├── __init__.py
│   ├── ground_truth_publisher.py
│   ├── mock_detection_publisher.py
│   ├── visualiser.py
│   ├── detector_node.py               # template for a real detector
│   └── evaluator.py
├── launch/
│   └── tracking_test.launch.py
├── dataset/                           # (example) your images + labels
│   ├── output_images/
│   └── labeled_data_output_images_edited.json
├── resource/tracking_test             # required by ament
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Installation

```bash
# inside an existing ROS 2 workspace
cd ~/ros2_ws/src
git clone <this-repo-url> tracking_test
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select tracking_test
source install/setup.bash
```

### Runtime Dependencies

```
rclpy            sensor_msgs
vision_msgs      geometry_msgs
message_filters  cv_bridge         # for image conversion
numpy            opencv‑python
```

Install Python wheels with `pip` if rosdep misses any.

---

## Dataset 

The dataset **is already provided with this repository** in:
```
tracking_test/dataset/
```
It contains:
- `labeled_data_output_images_edited.json` — Labeled bounding box data.
- `output_images/` — Corresponding image frames.

No dataset preparation is required.

---

## Nodes

### 1. Ground Truth Publisher
Publishes the dataset images and their ground-truth bounding boxes.

**Topic outputs:**
- `/image_raw` (`sensor_msgs/Image`)
- `/ground_truth` (`vision_msgs/Detection2DArray`)

### 2. Mock Detection Publisher
Simulates the output of a real object detector for testing and evaluation.

**Topic outputs:**
- `/detections` (`vision_msgs/Detection2DArray`)

This can be replaced with a real detector by publishing to the `/detections` topic.

### 3. Evaluator
Subscribes to both `/ground_truth` and `/detections` and computes:
- True Positives (TP)
- False Positives (FP)
- False Negatives (FN)
- Precision (P)
- Recall (R)
- ID-switches (for tracking)

Prints metrics every 50 frames and a final summary at the end.

### 4. Visualiser
Subscribes to `/image_raw`, `/ground_truth`, and `/detections`, and publishes an annotated image to `/eval_viz` showing:
- Ground truth boxes (e.g., green)
- Detection boxes (e.g., red)
- IDs for tracking

View in:
```bash
ros2 run rqt_image_view rqt_image_view /eval_viz
```

---

## Message conventions

All detections use **vision_msgs/Detection2DArray**:

* `results[0].hypothesis.class_id = 1`  → class *“human”*  
* `detection.id = "<track‑id>"`          → numeric ID as string  
* Box is `(cx, cy, w, h)` in pixels.

---

## Launch usage

### Default (mock detector)

```bash
ros2 launch tracking_test tracking_test.launch.py
```

The launch file starts:

1. The **mock detection publisher** (or your real detector)
2. The **evaluator**
3. The **visualiser**
4. The **ground truth publisher**
5. Optionally opens `rqt_image_view`

Edit `launch/tracking_test.launch.py` to:

* switch `mock_detection_publisher` → `detector_node`
* adjust dataset paths, delays, or detector parameters.

---

## Writing your own detector

Open `tracking_test/detector_node.py` and replace the three **TODO** blocks:

1. **Load model** (e.g. YOLO, Faster‑RCNN, TensorRT engine, …)  
2. *(optional)* initialise your tracker (ByteTrack, DeepSORT, …)  
3. Run inference in `run_model()` – return a list of  
   `(class_id, conf, x1, y1, x2, y2, track_id)` per frame.

Publish rate need not match ground‑truth; evaluator synchronises by
timestamp (50 ms slop).

---

## Evaluator output

Every `PRINT_EVERY` frames (default 50):

```
After 50 frames:  TP=172  FP=0  FN=10  P=1.000  R=0.945  ID-switches=16
```

When ground‑truth ends you’ll also see:

```
FINAL: TP=1189  FP=8  FN=68  P=0.993  R=0.946  ID-switches=71
```

---

## License

This project is licensed under the MIT License.
