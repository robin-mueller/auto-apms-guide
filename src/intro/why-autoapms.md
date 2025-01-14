---
order: 0
---
# Why AutoAPMS?

Why should you use this software and what is it for?

## Use Case

With AutoAPMS you've found a flexible, ROS2-based solution for designing robotic behaviors and creating highly automated missions. It aims to reduce the amount of manual work required when creating mission plans for nominal as well as abnormal operation. AutoAPMS's modular design allows software components to be reused effectively and introduces a powerful systematic behavior planning approach based on a high level of abstraction. This software adds a top-level deliberation layer to existing mission control solutions provided by [Nav2](https://nav2.org/) or [PX4](https://px4.io/). These kinds of software stacks commonly provide functionality on the execution level. AutoAPMS, however, is designed to orchestrate the underlying processes from the management level, effectively increasing the intelligence of an autonomous system and therefore it's capability to operate resiliently.

## Features

If you're looking for a C++ robot behavior design framework, that helps focusing your development efforts and minimizing the time required to create a functioning real-time system, you should incorporate AutoAPMS into your workspace. This package offers the following key features:

- Generic ROS 2 node design for implementing real-time tasks
- Modular, plugin-based approach for implementing robotic skills/actions
- Standardized, highly configurable behavior tree executor
- Powerful behavior tree builder API
- Automated contingency and emergency management system
- Useful command line tools for running and orchestrating missions

## What makes it special?

AutoAPMS combines the concepts introduced by multiple repositories. From all related projects, [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) is by far the most prominent one. This repo provides the core implementation of the behavior tree paradigm. However, when using it standalone, the application specific integration into the workspace is still up to the user. With AutoAPMS, all supported applications are building on ROS 2 which allows extending the behavior tree API to integrate well with `ament_cmake` and ROS's plugin system.

The behavior tree paradigm has also been adapted by [`nav2_behavior_tree`](https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree). Therefore, ROS's most popular navigation framework offers another well maintained repository that simplifies creating missions using behavior trees. Compared to the `nav2_behavior_tree` package, AutoAPMS thinks behavior trees further. It is more flexible, user friendly and automates/standardizes most of the troublesome and error-prone software development work. Most importantly, AutoAPMS takes behavior development one step further and additionally provides the user with a high-level mission management concept based on current research in this field.

> [!TIP]
> You may also include both Nav2's and AutoAPMS's packages in your workspace. In fact, Nav2 has plenty of useful features that AutoAPMS doesn't offer (the same goes for the other way around). It's important to understand that these two frameworks don't oppose but complement each other.
