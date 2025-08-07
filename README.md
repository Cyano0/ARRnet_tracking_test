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

All communication uses **standard** messages
(`sensor_msgs/Image`, `vision_msgs/Detection2DArray`) – no custom `.msg`
files needed.

---

## Directory layout

```
tracking_test/
├── tracking_test/                     # Python module
│   ├── __init__.py
│   ├── ground_truth_publisher.py
│   ├── mock_detection_publisher.py
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

## Dataset format

JSON excerpt:

```json
{
  "Timestamp": 1730892897.0809238,
  "File": "1730892897_080923740.png",
  "Labels": [
    { "Class": "human1", "BoundingBoxes": [85.05, 58.85, 102, 214] },
    { "Class": "human3", "BoundingBoxes": [457.8, 26.3, 107, 281] }
  ]
}
```

* `File` ‑ image file relative to `image_dir`
* `BoundingBoxes` ‑ `[x, y, width, height]` (pixels)
* `Class` must encode a numeric track‑ID (`human3` → ID 3).

---

## Nodes

| Executable | Description | Key Parameters |
|------------|-------------|----------------|
| **ground_truth_publisher** | Publishes `/image_raw` and `/ground_truth` from a JSON label file. | `json_path`, `image_dir`, `publish_rate` |
| **mock_detection_publisher** | Generates noisy detections and tracking IDs for testing. | `pos_jitter_px`, `size_jitter_pct`, `drop_prob`, `id_switch_prob` |
| **detector_node** | **Template** – plug in your real model / tracker. Publishes `/detections`. | `model_path`, `conf_threshold`, `nms_threshold` |
| **evaluator** | Subscribes to GT & detections, prints precision/recall + ID‑switch stats. | `IOU_THRESHOLD`, `PRINT_EVERY` (constants in code) |

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

1. `mock_detection_publisher`
2. `evaluator`   (after 1 s)
3. `ground_truth_publisher` (after 3 s, with dataset parameters)

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

MIT.  See `LICENSE` file.
