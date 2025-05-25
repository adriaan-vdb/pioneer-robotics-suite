# Pioneer Robotics Suite

A modular, ROS 2-based robotics software stack for the Pioneer 3-AT (P3AT) mobile robot platform. This suite provides autonomous navigation, teleoperation, sensor integration, vision-based object detection, and a modern GUI for status monitoring. The system is containerized with Docker and orchestrated via Docker Compose for reproducible deployment on real or simulated robots.

---

## Features

- **Autonomous Navigation:** SLAM, path planning, and goal management using ROS 2 Navigation Stack.
- **Teleoperation:** Joystick-based manual control with safety and emergency stop features.
- **Sensor Integration:** LIDAR, camera, IMU, and other sensors for robust perception.
- **Vision System:** Object and digit recognition, marker detection, and obstacle detection using deep learning and EasyOCR.
- **Custom ROS 2 Messages:** For object info, distances, and target goals.
- **Incident Recording:** Snapshot and bag recording for incident analysis.
- **GUI:** Real-time robot status display with pose, mode, emergency state, and messages.
- **Visualization:** RViz plugins for overlaying text and circular gauges.
- **Containerized Deployment:** Docker and Docker Compose for easy setup and multi-container orchestration.

---

## Directory Structure

```
AUTO4508-Project/
├── Dockerfile                # Docker build for ROS 2 and dependencies
├── build.sh                  # Build script for ROS 2 workspace
├── compose.yaml              # Docker Compose orchestration
├── ros_entrypoint.sh         # Entrypoint for ROS containers
├── src/                      # ROS 2 packages
│   ├── p3at_bringup/         # Launch, config, maps, URDF, RViz
│   ├── p3at_controller/      # Joystick/manual control, safety, emergency stop
│   ├── p3at_cpp_nodes/       # C++ nodes: navigation, exploration, goal client
│   ├── p3at_interface/       # Custom message definitions
│   ├── p3at_vision/          # Vision nodes: object/digit recognition, GUI
│   ├── ariaNode/             # Low-level hardware interface
│   └── rviz_2d_overlay_plugins/ # RViz overlay plugins
└── ...
```

---

## Getting Started

### Prerequisites
- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/)
- (Optional) ROS 2 Humble on host for development

### Build and Run (Recommended: Docker Compose)

1. **Clone the repository:**
   ```sh
git clone <this-repo-url> pioneer-robotics-suite
cd pioneer-robotics-suite
   ```
2. **Build and launch all services:**
   ```sh
docker compose build
docker compose up
   ```
3. **Access the GUI and RViz:**
   - The GUI will launch in a containerized X11 window (ensure X11 forwarding is enabled if remote).
   - RViz can be accessed via the `p3at_display` service.

### Manual Build (for developers)

```sh
./build.sh
source install/setup.bash
```

---

## Usage

### Launching Core Services
- **Navigation:**
  - SLAM: `ros2 launch p3at_bringup slam.launch.py`
  - Navigation: `ros2 launch p3at_bringup navigation.launch.py`
- **Teleoperation:**
  - Joystick: `ros2 launch p3at_controller controller.launch.py`
- **Vision:**
  - DepthAI: `ros2 launch depthai_ros_driver rgbd_pcl.launch.py parent_frame:=cam_frame`
  - Vision nodes: see `p3at_vision` package
- **Incident Recording:**
  - `ros2 bag record --snapshot-mode -a`

### GUI
- Real-time robot status display (pose, mode, emergency state, messages)
- Launch: `ros2 run p3at_vision gui`

### Visualization
- RViz overlays for text and circular gauges (see `rviz_2d_overlay_plugins`)

---

## Custom ROS 2 Messages
- `p3at_interface/msg/Distance.msg`: LIDAR distances and angles
- `p3at_interface/msg/ObjectInfo.msg`: Object position and description
- `p3at_interface/msg/TargetGoal.msg`: Target goal pose
- `p3at_vision/msg/ObjectInfo.msg`: Vision-detected object info

---

## Contributing
Pull requests and issues are welcome! Please follow ROS 2 and Python/C++ best practices. See the [CONTRIBUTING.md](CONTRIBUTING.md) if available.

## License
Specify your license here (e.g., MIT, Apache 2.0, etc.)

## Maintainers
- Main: [Your Name/Team]
- Email: [your@email.com]

---

## Repository Name Recommendation
**pioneer-robotics-suite**

This name reflects the platform, is professional, and avoids course/unit codes.
