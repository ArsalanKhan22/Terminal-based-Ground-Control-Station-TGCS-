# 🛫 TGCS - Terminal Ground Control Station for PX4

A terminal-based Ground Control Station (GCS) for **PX4** drones, implemented in Python using **MAVROS** and **ROS**. TGCS allows real-time command-line control, telemetry monitoring, and waypoint navigation of autonomous drones in both simulation and real-world scenarios.

---

## 🚀 Features

- Command-line interface for drone control and telemetry
- Arm / Disarm drone
- Set PX4 flight modes (OFFBOARD, AUTO, etc.)
- Takeoff and land to GPS-defined home positions
- Upate Home Location using current location 
- Waypoint queue and autonomous navigation
- Altitude hold and GPS-based target movement
- Real-time position, velocity, and battery telemetry
- Command logging with timestamps
- Manual local movement using arrow keys (keyboard control)
- Works with SITL, Gazebo, or real drones

---

## 📁 Repository Contents

- `tgcs.py` - Main TGCS implementation (ROS + MAVROS node)
---

## ⚙️ Requirements

- **Operating System**: Ubuntu 18.04 or 20.04 (recommended)
- **ROS**: Melodic / Noetic
- **PX4 Firmware**: with MAVROS support
- **Python**: ≥ 3.6
- **Dependencies**:
  - `rospy`
  - `mavros_msgs`
  - `geometry_msgs`, `sensor_msgs`
  - `pymap3d`
  - `scipy`


Install ROS dependencies:
```bash
sudo apt install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras
rosrun mavros install_geographiclib_datasets.sh
```

Run in Simulation (Gazebo or SITL):
```bash
roslaunch px4 mavros_posix_sitl.launch
rosrun {Directory} tgcs.py
```

## 📚 Acknowledgements

- [PX4 Autopilot](https://px4.io)
- [MAVROS](http://wiki.ros.org/mavros)
- [ROS (Robot Operating System)](https://www.ros.org/)
- [pymap3d](https://github.com/geospace-code/pymap3d)
- [SciPy](https://scipy.org/)
- [GeographicLib](https://geographiclib.sourceforge.io/)
