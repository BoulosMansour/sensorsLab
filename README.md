# Sensors and robot localization in ROS 2

Coursework from the robotics sensors labs at Politecnico di Torino, written in Python with ROS 2. The main piece is an **Extended Kalman Filter** that localizes a differential-drive robot from landmark measurements.

## Robot localization project (`robot_localization_project/`)

An EKF node that estimates the robot pose `[x, y, θ]` and compares it with the simulator's ground truth.

- **Motion models:** velocity model (from `/cmd_vel`) and odometry model (from `/diff_drive_controller/odom`), run as two parallel filters.
- **Measurement model:** range and bearing to known landmarks, simulated with a limited field of view (45°) and range (8 m) plus Gaussian noise.
- **Outputs:** filtered estimates on `ekf_odom` and `ekf_vel`, logged with their covariances; `compute_metrics.py` computes the error against ground truth and plots the trajectories and uncertainty ellipses (`Figure_1.png` … `Figure_6.png`).
- **Configuration:** landmarks, initial pose and noise parameters in `config/ekf.yaml`; launch with `launch/ekf.launch.py`.

```bash
colcon build --packages-select robot_localization_project
source install/setup.bash
ros2 launch robot_localization_project ekf.launch.py
```

## Labs

| Folder | Topic |
| --- | --- |
| `lab01_pkg` | First ROS 2 nodes: publisher/subscriber controller and localization |
| `lab02_interfaces`, `lab02_pkg` | Custom services and actions (trajectory computation, move-distance server) |
| `lab03_interfaces`, `lab03_pkg` | Action-based motion: move forward, rotate to an absolute heading, trajectory execution |
| `lab04` | EKF fundamentals in a Jupyter notebook |
| `lab05` | Localization with recorded rosbags |
| `lab06` | Maps of the simulated worlds |

## Requirements

ROS 2, Python 3, NumPy, Matplotlib, `tf_transformations`.
