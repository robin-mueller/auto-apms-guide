---
order: 30
---
# Deploying Behaviors

In this tutorial we're going to describe everything that you need to know when using AutoAPMS's [behavior tree executor](../concept/behavior-tree-executor.md).

For deploying a behavior you need to [build a behavior tree](./building-behavior-trees.md) beforehand. Read the corresponding tutorial if you're not familiar with this process.

## Using a Suitable Build Handler

To define how the executor builds the behavior tree to be executed, you must tell it which build handler implementation to use. This can be done by in two ways:

- Setting the executor's ROS 2 parameter named `build_handler`.
- Using the `build_handler` field of a [`StartTreeExecutor`](../concept/behavior-tree-executor.md#starttreeexecutor) action goal.

The user is expected to provide a resource identity in order to specify the build handler to be used. Visit [this page](../concept/common-resources.md#behavior-tree-build-handlers) to learn more about build handler resources and the standard implementations we provide.

## Spinning the Executor

Users may spawn the behavior tree executor as it's done with any other ROS 2 node. We provide a minimal executable called `tree_executor` which simply spins the node. It can be used from the terminal or inside a [ROS 2 launch file](https://docs.ros.org/en/humble/tutorial/Intermediate/Launch/Creating-Launch-Files.html) like this:

::: code-group

```bash [Terminal]
ros2 run auto_apms_behavior_tree tree_executor
```

```py [launch.py]
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="auto_apms_behavior_tree",
                executable="tree_executor"
            )
        ]
    )
```

:::

If you're using [ROS 2 Composition](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html), the corresponding component can be found under the name `auto_apms_behavior_tree::TreeExecutorNode` and spawned like this:

::: code-group

```bash [Terminal]
# Spawn the component container
ros2 run rclcpp_components component_container

# Add the executor node to the container (from another terminal)
ros2 component load /ComponentManager auto_apms_behavior_tree auto_apms_behavior_tree::TreeExecutorNode
```

```py [launch.py]
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription(
        [
            ComposableNodeContainer(
                name="my_container",
                namespace="my_namespace",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="auto_apms_behavior_tree", 
                        plugin="auto_apms_behavior_tree::TreeExecutorNode"
                    )
                ]
            )
        ]
    )
```

:::

The package `auto_apms_behavior_tree` additionally offers an executable called `run_behavior` which automatically spawns the executor and starts executing a specific behavior tree directly afterwards. It implements a custom runtime and handles SIGINT signals appropriately. Use it like this:

::: code-group

```bash [Terminal]
ros2 run auto_apms_behavior_tree run_behavior -h  # Prints help information
```

```py [launch.py]
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="auto_apms_behavior_tree",
                executable="run_behavior",
                arguments=["-h"]  # Prints help information
            )
        ]
    )
```

:::

It accepts one argument: The build request for the underlying build handler. By default, you can simply pass a [behavior tree resource identity](../concept/common-resources.md#behavior-trees) and the corresponding behavior tree will be executed immediately. This is because the behavior executor loads `TreeFromResourceBuildHandler` on startup. You can customize which build handler should be loaded initially by setting the corresponding ROS 2 parameter like this:

::: code-group

```bash [Terminal]
ros2 run auto_apms_behavior_tree run_behavior "<build_request>" --ros-args -p build_handler:=my_namespace::MyBuildHandlerClass
```

```py [launch.py]
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="auto_apms_behavior_tree",
                executable="run_behavior",
                arguments=["<build_request>"],
                parameters=[{
                    "build_handler": "my_namespace::MyBuildHandlerClass"
                }]
            )
        ]
    )
```

:::

::: info
If you cannot use `run_behavior` for some reason (e.g. when applying ROS 2 composition), you should first spawn the node and then, in a separate step, send a [`StartTreeExecutor`](../concept/behavior-tree-executor.md#starttreeexecutor) action goal.

As a last resort, you can also pass a build request as the first command line argument of `tree_executor` which populates the `arguments` member of `rclcpp::NodeOptions`. This will also cause the executor to automatically start ticking the behavior tree but the runtime doesn't handle interrupt signals very well.
:::

## Interacting with the Executor

Users may communicate with a spinning executor node by creating ROS 2 action clients and sending goals to the corresponding actions:

- [`StartTreeExecutor`](../concept/behavior-tree-executor.md#starttreeexecutor)
- [`CommandTreeExecutor`](../concept/behavior-tree-executor.md#commandtreeexecutor)

The former action allows to initiate the execution of a particular behavior tree and the latter offers a way to interrupt/restart this process.

We conveniently provide behavior tree nodes associated with these actions:

- [`StartExecutor`](../reference/behavior-tree-nodes.md#startexecutor)
- [`ResumeExecutor`](../reference/behavior-tree-nodes.md#resumeexecutor)
- [`PauseExecutor`](../reference/behavior-tree-nodes.md#pauseexecutor)
- [`HaltExecutor`](../reference/behavior-tree-nodes.md#haltexecutor)
- [`TerminateExecutor`](../reference/behavior-tree-nodes.md#terminateexecutor)

Since a behavior tree executor implements the same standard interfaces as any other ROS 2 node, it's also possible to query or manipulate parameters at runtime. Besides the statically declared [executor parameters](../concept/behavior-tree-executor.md#configuration-parameters) you may additionally use [scripting enum parameters](../concept/behavior-tree-executor.md#scripting-enums) or [global blackboard parameters](../concept/behavior-tree-executor.md#global-blackboard).

The following nodes allow for interacting with ROS 2 parameters from within a behavior tree:

- [`HasParameter`](../reference/behavior-tree-nodes.md#hasparameter)
- [`GetParameter`](../reference/behavior-tree-nodes.md#getparameter)
- `GetParameter<TypeName>` (implemented for each ROS 2 parameter type)
- [`SetParameter`](../reference/behavior-tree-nodes.md#setparameter)
- `SetParameter<TypeName>` (implemented for each ROS 2 parameter type)

## Nesting Behaviors

The flexible action interface introduced by the behavior tree executor makes it possible to deploy behaviors remotely. This architecture also allows for multiple behavior trees being executed in real parallelism by different executor nodes communicating with each other. Therefore, this functionality empowers users of AutoAPMS to design arbitrarily complex systems and spawn deeply nested behaviors trees.

By using `StartExecutor` and the other behavior tree nodes designed for interacting with `TreeExecutorNode`, the user is not limited to including subtrees locally but profits significantly from increased modularity. With the `attach` field, you can control whether to attach to a behavior tree and wait for it to be completed or detach one behavior from another. In detached mode, you're able to do work asynchronously but must manually make sure to not leave the remote executor running after the parent behavior completed (for example by ticking `TerminateExecutor` last thing).

The next tutorial explains how nested behaviors are used in AutoAPMS to create reactive behaviors. We specifically organize multiple behavior trees and create an even higher level of abstraction: Missions.
