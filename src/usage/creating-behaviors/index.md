---
order: 10
sidebar: 2 Creating Behaviors
---
# Creating Behaviors

::: info
The following assumes that you are familiar with the concept of behavior trees. If not, make sure to read more about [behavior trees within AutoAPMS](../../concepts/behavior-trees.md) before you continue.
:::

We incorporate behavior trees for modeling transitions between individual skills or tasks (both terms are used interchangeably) and tell the robot how to achieve a certain goal. Behavior trees are composed of nodes that can be considered clients to a specific function. They may for example request to execute a skill/task in order to query information or perform an action. So before it makes sense to think about creating behavior trees, you must implement ROS 2 nodes that act as servers for the required functionality and do the actual work. Refer to our [guidelines](../implementing-skills/) for implementing these.

While all tree nodes that are related to ROS 2 must work asynchronously, other nodes may implement functions that do synchronous calculations. However, you should think carefully when integrating synchronous nodes and make sure that they don't prevent other nodes from being executed for too long.

There are various predefined nodes that you should feel encouraged to use. Learn more about those and how to implement custom nodes [here](./implement-nodes.md). Once you have gathered all your behavior tree nodes, you may [build your behavior trees](./build-trees.md) and [execute them using ROS 2](../deploying-behaviors/).
