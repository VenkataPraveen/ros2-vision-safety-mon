
# ros2-vision-safety-mon
ROS 2 camera-based safety monitor with motion detection, C++ watchdog, and fault injection — built on Raspberry Pi
=======
# ROS 2 Vision Safety Monitor with Fault Injection

A ROS 2 system running on Raspberry Pi that demonstrates real-time camera-based
motion detection, a C++ safety watchdog, and fault injection for testing.

## What It Does
- Publishes webcam frames as ROS 2 Image messages (Python)
- Detects motion using OpenCV frame differencing (Python)
- Monitors the pipeline for faults using a C++ watchdog node
- Injects controlled faults (dropout, stuck-value, noise) to verify detection logic
<img width="581" height="602" alt="image" src="https://github.com/user-attachments/assets/5b6382a4-6652-4fd5-93a7-330548692768" />

## Fault Types Detected
| Fault | Description | Detection Method |
|-------|-------------|-----------------|
| Dropout | No messages arriving | Timestamp timeout (3s) |
| Stuck Value | Same score repeated | Value variance over 5 ticks |
| Noisy | Random invalid data | Score range validation |

## Hardware
- Raspberry Pi 5 (Debian Trixie, aarch64)
- USB Webcam

## Software Stack
- ROS 2 Jazzy (via Docker)
- Python 3, OpenCV, cv_bridge
- C++17

## Package Structure
- `vision_monitor` — camera publisher, motion detector, fault injector (Python)
- `vision_monitor_msgs` — custom MotionStatus message definition
- `vision_watchdog` — safety watchdog node (C++)

## Key Concepts Demonstrated
- ROS 2 publisher/subscriber pattern
- Custom ROS 2 message types
- Mixed Python/C++ ROS 2 workspace
- Fault injection methodology (ISO 26262 inspired)
- Distributed topic communication across two machines

## Build and Run
```bash
cd ros2_ws
colcon build
source install/setup.bash

# Terminal 1 - Camera
ros2 run vision_monitor camera_publisher

# Terminal 2 - Motion Detection
ros2 run vision_monitor motion_detector

# Terminal 3 - Watchdog
ros2 run vision_watchdog watchdog_node

# Terminal 4 - Inject a fault
ros2 run vision_monitor fault_injector stuck
```
