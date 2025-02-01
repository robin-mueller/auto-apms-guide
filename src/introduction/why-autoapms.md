---
order: 0
---
# Why AutoAPMS?

As you might already know, designing a robotic real-time system and enabling it to manipulate or move around its environment is a painfully time consuming task and requires a lot of practical experience. Shortly after you started prototyping, you'll probably realize that a big portion of what you do isn't actually related to your application. Especially when it becomes more complex, there are a lot of "side quests" which revolve around structuring functionality, maintaining modularity and most importantly defining opertional fallback mechanisms if something goes wrong.

"If only there was another one of those helpful ROS 2 packages to provide me with a framework to deal with all that", you might think. 🤔

Enter **AutoAPMS**: A ROS 2 project that offers a flexible system architecture designed for helping you focus on the most important development tasks. It provides an end-to-end solution for creating custom robotic applications allowing you to prototype faster and make your operation safer.

## Use Case

With AutoAPMS you've found a flexible, ROS2-based solution for designing robotic behaviors and creating highly automated applications. It aims to reduce the amount of manual work required when creating behaviors for nominal as well as abnormal operation. AutoAPMS's modular design allows software components to be reused effectively and introduces a powerful systematic behavior planning approach based on a high level of abstraction. This software adds a top-level deliberation layer to existing mission control solutions provided by [Nav2](https://nav2.org/) or [PX4](https://px4.io/). These kinds of software stacks commonly provide functionality on the execution level. AutoAPMS, however, is designed to orchestrate the underlying processes from the management level, effectively increasing the intelligence of an autonomous system and therefore it's capability to operate resiliently.

## Features

AutoAPMS provides developers of real-time systems with the following key features:

- Tools and methods for designing cooperative systems
- Modular, plugin-based approach for implementing robotic skills/tasks
- User-friendly task planning system adopting the behavior tree paradigm
- Standardized, highly configurable behavior tree executor
- Powerful behavior tree builder API
- Automated contingency and emergency management system
- Useful command line tools for running and orchestrating missions

## What makes it special?

This project combines the concepts introduced by multiple other repositories. From all related projects, [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) is by far the most prominent one. This repo provides the core implementation of the behavior tree paradigm. However, when using it standalone, the application specific integration into the workspace is still up to the user. With AutoAPMS, all supported applications are building on ROS 2 which allows extending the behavior tree API to integrate well with `ament_cmake` and ROS's plugin system.

The behavior tree paradigm has also been adapted by [Nav2 Behavior Tree](https://docs.nav2.org/behavior_trees/index.html). Therefore, ROS's most popular navigation framework offers another well maintained repository that simplifies creating missions using behavior trees. Compared to the `nav2_behavior_tree` package, AutoAPMS thinks behavior trees further. It is more flexible, user friendly and automates/standardizes most of the troublesome and error-prone software development work. Additionally, It takes behavior development to the next step and provides the user with a high-level mission management concept based on current research in this field.

::: tip
You may also include both Nav2's and AutoAPMS's packages in your workspace. In fact, Nav2 has plenty of useful features that AutoAPMS doesn't offer (the same goes for the other way around). It's important to understand that these two frameworks don't oppose but complement each other.
:::
