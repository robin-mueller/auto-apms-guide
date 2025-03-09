---
order: 0
---
# Getting Started

Learn how to download and install the software and get started with your first simulation. Also make sure to check out [additional software](./additional-software.md) that you might be interested in when developing with AutoAPMS.

## Hardware Compatibility

AutoAPMS is currently only available on **Linux**.

| ROS 2 Version | OS | Status |
| :-------------: | :-----------: | :-----------: |
| [Humble Hawksbill](https://docs.ros.org/en/humble/index.html) | [Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/) | [![ROS 2 Humble Test](https://github.com/robin-mueller/auto-apms/actions/workflows/humble.yaml/badge.svg)](https://github.com/robin-mueller/auto-apms/actions/workflows/humble.yaml) |

## Setting up the Workspace

The following installation guide helps you getting started with AutoAPMS by building it from source. Finally, you may test your installation by running an example.

Firstly, you have to create a [ROS 2 workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) and clone this repository.

```bash
mkdir ros2_ws && cd ros2_ws
(mkdir src && cd src && git clone https://github.com/robin-mueller/auto-apms.git)
```

Afterwards, install all required dependencies. We assume that you already installed ROS 2 on your system.

```bash
rosdep init  # Skip this if rosdep has already been initialized
rosdep update
rosdep install --from-paths src --ignore-src -y

# Python packages for simulation (not all are available with rosdep)
python3 -m pip install -r src/auto-apms/auto_apms_simulation/requirements.txt
```

Then, build and install all of the source packages up to `auto_apms_examples`.

> [!NOTE]
> We highly recommend building using the `symlink-install` option since AutoAPMS extensively utilizes XML and YAML resources. This option installs symbolic links to those non-compiled source files meaning that you don't need to rebuild again and again when you're for example tweaking a behavior tree document file. Instead, your changes take effect immediately and you just need to restart your application.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to auto_apms_examples --symlink-install
```

Congratulations, you've already successfully installed all necessary resources.

## Launch an Example

You may now launch a lightweight simulation that applies the concepts offered by AutoAPMS. This should give you an idea of what's possible with this framework.

The basic robot behavior can be observed by executing

```bash
source install/setup.bash
ros2 launch auto_apms_examples pyrobosim_hogwarts_launch.py
# Press Ctrl+C to quit
```

The actions of each robot you've seen are executed using behavior trees. This functionality is provided by the `auto_apms_behavior_tree` package. However, each robot is acting independently and they are not aware of their environment. Yet.

Now, we want to make the robots more intelligent and allow them to dynamically adjust their behavior when they encounter other robots inside one of the hallways. This is realized by implementing fallback mechanisms introduced by the `auto_apms_mission` package. To achieve that, you simply have to specify the following launch argument.

```bash
source install/setup.bash
ros2 launch auto_apms_examples pyrobosim_hogwarts_launch.py mission:=true
# Press Ctrl+C to quit
```

The robots dynamically decide to retreat and wait until the hallway they are about to cross is not occupied anymore. They basically monitor if a certain event occurs and initialize a certain sequence of action if applicable. With this, we effectively introduced automatically orchestrated reactive behaviors.

<video autoplay controls>
  <source src="https://github.com/user-attachments/assets/adbb7cab-1a9b-424b-af61-61c351986287" type="video/mp4">
Your browser does not support the video tag.
</video>
