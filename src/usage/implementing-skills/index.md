---
order: 0
sidebar: 1 Implementing Skills
---
# Implementing Skills

Robotic applications are realized by performing specific tasks in a given order. ROS 2 allows the user to implement [actions](https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html), [services](https://docs.ros.org/en/humble/Concepts/Basic/About-Services.html) and simple [topic](https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html) publishers or subscribers. With these concepts, robots are able to execute arbitrarily complex jobs. However, it is important to think about what a robot should be capable of and how these skills should be accessed before starting to write software. With AutoAPMS, it's crucial to have a well designed fundamental layer of execution since its higher level concepts like behavior trees and missions systematically build upon the lower level functionality. Developing the [ROS 2 nodes](https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html) required for your specific application is probably the most complex development task. Here are some generally applicable guidelines:

- **Keep it simple**

  It's better to have fewer nodes and interfaces. You should rather enable using actions or services in many ways, but keep it intuitive for the user.

- **Separate responsibilities**

  You should achieve a high level of modularity within your system. This means that you need to implement nodes that serve a specific purpose, but one purpose only.

- **Write reusable code**

  It's desirable to distinguish between core functionality and ROS 2 specific code. Reduce the amount of local calculation and keep your ROS 2 nodes lightweight.

AutoAPMS provides a convenient [action server wrapper](https://robin-mueller.github.io/auto-apms/classauto__apms__util_1_1ActionWrapper.html) that you may inherit from to create custom robot skills. It helps you with the troublesome process of writing asynchronous functions and simplifies handling incoming requests. You simply have to overwrite the virtual methods according to the documentation.
