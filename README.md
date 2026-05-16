# Smart Trash Tracking System

A Raspberry Pi 5 robotics project for a four-mecanum-wheeled robot that detects garbage with an IMX219 camera, follows the detected target, and reacts with a burst movement when garbage is thrown.

The current ROS 2 workspace contains the vision, trajectory, and motor-control nodes for the robot. The main detection path uses a YOLO model to publish normalized target coordinates, while the trajectory node converts target motion into mecanum movement bursts.

## Demo

Add the demo video here when it is ready.

Suggested options:

- Upload the video to GitHub and paste the generated link below.
- Place a local video at `media/demo.mp4` and replace this placeholder with a link or video tag.

```md
[Demo video](media/demo.mp4)
```

## Hardware

- Raspberry Pi 5
- IMX219 camera module
- Four mecanum wheels
- Four DC motors with motor drivers
- GPIO motor wiring as configured in `src/trash_script/trash_script/move.py`

## Software Stack

- ROS 2
- Python 3
- OpenCV and cv_bridge
- Ultralytics YOLO
- lgpio for Raspberry Pi GPIO control
- `camera_ros` for publishing the IMX219 camera image topic

## Repository Layout

```text
.
|-- src/
|   |-- trash_script/
|   |   |-- trash_script/
|   |   |   |-- vision.py        # YOLO detection and tracker node
|   |   |   |-- new_vision.py    # HSV/red-target test vision node
|   |   |   |-- trajectory.py    # Follow and throw-intercept decision node
|   |   |   `-- move.py          # Four-mecanum-wheel burst motor control
|   |   |-- best.pt              # YOLO model
|   |   |-- best2.pt             # YOLO model used by vision.py
|   |   `-- yolov8n.pt           # Fallback YOLO model
|   `-- yolo_node.py             # Older standalone YOLO detector
|-- media/                       # Put demo videos or GIFs here
`-- README.md
```

`install/`, `log/`, and other colcon output folders are generated artifacts. They can be rebuilt from source and should normally stay out of future commits.

## ROS Topics

| Topic | Type | Publisher | Subscriber | Purpose |
| --- | --- | --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | camera node | `vision.py`, `new_vision.py` | Camera frames from the IMX219 |
| `/target_coord` | `geometry_msgs/PointStamped` | `vision.py` or `new_vision.py` | `trajectory.py` | Normalized target position and size |
| `/vision/state` | `std_msgs/String` | `vision.py` | debug tools | `SEARCH` or `TRACK` state |
| `/yolo/debug_image` | `sensor_msgs/Image` | `vision.py` | debug tools | Detection overlay image |
| `/burst_cmd` | `std_msgs/String` | `trajectory.py` | `move.py` | Direction, duration, and optional timestamp |

For `vision.py`, `/target_coord.point.x` is horizontal error, `/target_coord.point.y` is vertical error, and `/target_coord.point.z` is normalized bounding-box area.

## Setup

Install ROS 2 and camera support on the Raspberry Pi, then install the Python dependencies used by the nodes:

```bash
sudo apt update
sudo apt install python3-opencv python3-lgpio
pip install ultralytics
```

From the workspace root:

```bash
cd Smart-Trash-Tracking-System
colcon build --symlink-install
source install/setup.bash
```

## Run

Start the camera node so `/camera/image_raw` is available. The exact command depends on how `camera_ros` is installed on the Raspberry Pi, for example:

```bash
ros2 launch camera_ros camera.launch.py
```

Run the YOLO vision node:

```bash
ros2 run trash_script vision
```

Run trajectory planning:

```bash
ros2 run trash_script trajectory
```

Run motor control on the Raspberry Pi:

```bash
ros2 run trash_script move
```

For early camera and tracking tests, `new_vision` can be used with a red target:

```bash
ros2 run trash_script new_vision
```

## Tuning Notes

- `vision.py` parameters: `conf_threshold`, `imgsz`, `yolo_every_n`, and `use_tracker`.
- `trajectory.py` contains follow thresholds, throw speed threshold, cooldown, and intercept duration.
- `move.py` contains GPIO pins, motor signs, left/right gain calibration, PWM frequency, braking duty, and timing scale.

Tune these values on the real robot, because camera angle, lighting, motor driver behavior, and floor friction strongly affect tracking and intercept movement.

## Models

The package installs `best.pt`, `best2.pt`, and `yolov8n.pt` into the ROS share directory. `vision.py` loads `best2.pt` by default and falls back to `yolov8n.pt` if needed.

## License

Apache-2.0. See `LICENSE`.
