---
order: 40
---
# PX4 Bridge

We allow controlling autonomous sytems running PX4 by incorporating the [PX4/ROS2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface.html) and allow the user to define custom modes that can be dynamically registered with the autopilot. These modes are written in ROS 2 and run externally while communicating with PX4 using the internal [uORB messages](https://docs.px4.io/main/en/middleware/uorb.html).

- ModeExecutor
- ModeExecutorFactory
