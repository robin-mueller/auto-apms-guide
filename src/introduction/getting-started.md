---
order: 20
---
# Getting Started

Start leveraging the advantages of Behavior Trees 🌳 fully integrated with ROS 2 🤖.

> [!NOTE]
> Currently we support **Linux only**!.

| ROS 2 | Build & Test |
|:------------------:|:------:|
| Jazzy Jalisco | [![Jazzy](https://github.com/autoapms/auto-apms/actions/workflows/jazzy.yaml/badge.svg?branch=master)](https://github.com/autoapms/auto-apms/actions/workflows/jazzy.yaml) |
| Kilted Kaiju | [![Kilted](https://github.com/autoapms/auto-apms/actions/workflows/kilted.yaml/badge.svg?branch=master)](https://github.com/autoapms/auto-apms/actions/workflows/kilted.yaml) |
| Rolling Ridley | [![Rolling](https://github.com/autoapms/auto-apms/actions/workflows/rolling.yaml/badge.svg?branch=master)](https://github.com/autoapms/auto-apms/actions/workflows/rolling.yaml) |

## Run your first Behavior

The following installation guide helps you getting started with AutoAPMS.

1. Create a [ROS 2 workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) and clone this repository

    ```bash
    mkdir ros2_ws && cd ros2_ws
    (mkdir src && cd src && git clone https://github.com/autoapms/auto-apms.git)
    ```

1. Install all required dependencies. We assume that you already installed ROS 2 on your system

    ```bash
    rosdep init  # Skip this if rosdep has already been initialized
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```

1. Build and install all packages required for `auto_apms_examples`

    ```bash
    colcon build --packages-up-to auto_apms_examples --symlink-install
    ```

1. Run your first behavior using `ros2 behavior`. This is an extension of the ROS 2 CLI introduced by the `auto_apms_ros2behavior` package

    ```bash
    source install/setup.bash
    ros2 behavior run auto_apms_examples::demo::HelloWorld --blackboard name:=Turtle
    ```

    ![auto-apms-gif](https://github.com/user-attachments/assets/0039aa09-9448-4102-9eb3-38138a805728)

## Visual Demonstration

Finally, you may as well run a cool **visual demonstration** of what's possible with this framework.

1. Install dependencies and build package `auto_apms_simulation`

    ```bash
    # Python packages for simulation (not all are available with rosdep)
    python3 -m pip install -r src/auto-apms/auto_apms_simulation/requirements.txt
    colcon build --packages-up-to auto_apms_simulation --symlink-install
    ```

1. Run the less intelligent behavior first

    ```bash
    source install/setup.bash
    ros2 launch auto_apms_simulation pyrobosim_hogwarts_launch.py
    # Press Ctrl+C to quit
    ```

    The actions of each robot you've seen are executed using behavior trees. This functionality is provided by the `auto_apms_behavior_tree` package. However, each robot is acting independently and they are not aware of their environment. Yet.

1. Now, we want to make the robots more intelligent and allow them to dynamically adjust their behavior when they encounter other robots inside one of the hallways. This is realized by implementing fallback mechanisms introduced by the `auto_apms_mission` package. To achieve that, add a launch argument

    ```bash
    source install/setup.bash
    ros2 launch auto_apms_simulation pyrobosim_hogwarts_launch.py mission:=true
    # Press Ctrl+C to quit
    ```

    The robots dynamically decide to retreat and wait until the hallway they are about to cross is not occupied anymore. They basically monitor if a certain event occurs and initialize a corresponding sequence of action if applicable. With this, we effectively introduced automatically orchestrated reactive behaviors.

    <video autoplay controls>
      <source src="https://github.com/user-attachments/assets/adbb7cab-1a9b-424b-af61-61c351986287" type="video/mp4">
    Your browser does not support the video tag.
    </video>
