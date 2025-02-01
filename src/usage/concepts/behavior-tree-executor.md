---
order: 20
---
# Behavior Tree Executor

Commonly required functionality for executing behavior trees within the ROS 2 context is provided by [`TreeExecutorNode`](https://robin-mueller.github.io/auto-apms/classauto__apms__behavior__tree_1_1TreeExecutorNode.html). This flexible ROS 2 node offers a universal interface compatible with any behavior tree which complies with [our definitions](./behavior-trees.md). It implements the behavior tree build handler concept allowing the user to define how the executor is supposed to create the desired behavior tree.

## ROS 2 Interfaces

There are two ROS 2 actions allowing the user to communicate with an executor node. They may also be used with custom executors created by inheriting from `TreeExecutorNode`.

### StartTreeExecutor

**Action Name:** `<node_name>/start`

**C++ Type:** `auto_apms_interfaces::action::StartTreeExecutor`

```cpp
#include "auto_apms_interfaces/action/start_tree_executor.hpp"
using StartTreeExecutor = auto_apms_interfaces::action::StartTreeExecutor;
```

**This action allows starting the execution of a specific behavior tree.**

In the request, the user must formulate which behavior tree is to be executed by providing `build_request`. Optionally, `build_handler` may be specified to determine which build handler must be used to interpret `build_request` (If empty, the one that is currently loaded will be used). Depending on the implementation of the build handler, `node_manifest` and `root_tree` may be required for registering the corresponding behavior tree nodes respectively to define the entry point for execution.`build_request`, `node_manifest` and `root_tree` are passed to [`TreeBuildHandler::setBuildRequest`](https://robin-mueller.github.io/auto-apms/classauto__apms__behavior__tree_1_1TreeBuildHandler.html#ac4c4887fcd65b024a89445009cbe15b4).

Only a single `StartTreeExecutor` goal is allowed to be executed at a time.

#### Fields {#start-fields}

| Name | Type | Default | Description |
| :---: | :---: | :---: | :--- |
| `build_request` | `std::string` | ❌ | String that encodes information about which behavior tree to execute. It must be formatted in a suitable way so that the underlying build handler is able to interpret it correctly. The user must be aware of which format is accepted by which build handler and always create appropriate requests. Otherwise, the build handler may throw an error and the action goal will be rejected. If the build handler doesn't implement that level of validation, it's undefined behavior. |
| `build_handler` | `std::string` | "" | Fully qualified class name of a declared behavior tree build handler plugin to be used for interpreting `build_request`. If empty, the one that is currently loaded by the executor will be used. May also be `none` for requesting to "unload" the current build handler (This requires derived executors to be able to build the behavior tree without relying on a build handler). |
| `root_tree` | `std::string` | "" | Name of the behavior tree to be considered the entry point of execution. This field may be empty if the underlying build handler is able to determine the root tree using `build_request` alone. Otherwise, the build handler may use this information appropriately when it is passed to `TreeBuildHandler::setBuildRequest`. |
| `node_manifest` | `std::string` | "" | User-defined node manifest (encoded) specifying how to register all of the behavior tree nodes required by `build_request`. This field may be empty if the underlying build handler is able to independently perform the node registration. Otherwise, the build handler may use this information appropriately when it is passed to `TreeBuildHandler::setBuildRequest`. |
| `node_overrides` | `std::string` | "" | User-defined node manifest (encoded) specifying optional registration parameters for previously registered nodes supposed to be replaced by other implementations. The executor uses this information to override node registration names after `TreeBuildHandler::buildTree` was called i.e. the behavior tree was created and before the execution routine is started. |
| `attach` | `bool` | `true` | Flag for determining the execution mode. If `true` (attached mode), the action will attach to the execution routine, run for as long as it is alive and return a goal result encapsulating the final status of the executed behavior tree. If `false`, the action will return immediately after the execution has been started allowing the client to do work asynchronously while the behavior tree is being executed. |
| `clear_blackboard` | `bool` | `true` | Flag for determining whether to clear the executor's global behavior tree blackboard before starting the execution or not. If `true` |

### CommandTreeExecutor

**Action Name:** `<node_name>/cmd`

**C++ Type:** `auto_apms_interfaces::action::CommandTreeExecutor`

```cpp
#include "auto_apms_interfaces/action/command_tree_executor.hpp"
using CommandTreeExecutor = auto_apms_interfaces::action::CommandTreeExecutor;
```

**This action allows to send certain commands for controlling the execution routine.**

If the executor has been started in detached mode (`attach` = false), this action provides a possibility to still be able to control it. `TreeExecutorNode` implements a state machine that allows the user to resume, pause and stop the behavior tree execution dynamically. For starting it, the user must always use `StartTreeExecutor`. Sending an action goal that encapsulated a specific control command only works if the targeted executor is actually executing a tree. Otherwise, the goal request is rejected.

Stopping or halting the execution of a behavior tree works by invoking `BT::Tree::haltTree` on the respective object.

::: warning
For `TreeExecutorNode` to be able to process `CommandTreeExecutor` action goals, all behavior tree nodes to be ticked must be implemented asynchronously. Make sure to read [this guide](https://www.behaviortree.dev/docs/guides/asynchronous_nodes#asynchronous-vs-synchronous) if you don't know what it means to "synchronously" or "asynchronously" execute a behavior tree.
:::

#### Fields {#cmd-fields}

| Name | Type | Default | Description |
| :---: | :---: | :---: | :--- |
| `command` | `uint8_t` | 0 | Enumeration for the command to be sent to the executor. By default, `COMMAND_UNDEFINED` is assigned to this field, so the user must explicitly set the desired command or the request will be rejected. |

The possible enumeration are defined as constants in the action interface definition:

| Constant | Value | Description |
| :---: | :---: | :--- |
| `COMMAND_UNDEFINED` | 0 | Invalid command. Will always be rejected. |
| `COMMAND_RESUME` | 1 | Transition from the idle state entered using `COMMAND_PAUSE` or `COMMAND_HALT` to the running state. |
| `COMMAND_PAUSE` | 2 | Pause the execution. This keeps the executor busy but prevents the behavior tree from being ticked again after this command was accepted. |
| `COMMAND_HALT` | 3 | Halt the behavior tree. This causes the execution routine to invoke `BT::Tree::haltTree` which stops the currently running behavior tree node. Afterwards, the executor is kept busy and the tree won't be ticked until `COMMAND_RESUME` is received. |
| `COMMAND_TERMINATE` | 4 | Halt the behavior tree and terminate the execution routine. If accepted, the executor invokes `BT::Tree::haltTree` and ultimately resets the execution routine. This enables the executor to accept new [`StartTreeExecutor`](#starttreeexecutor) goals. |

## Behavior Tree Build Handler

AutoAPMS introduces the concept of behavior tree build handlers which allows the user to write C++ algorithms for creating behavior trees according to certain rules. Users must inherit from `TreeBuildHandler` and override the following two virtual methods:

- [`TreeBuildHandler::setBuildRequest`](https://robin-mueller.github.io/auto-apms/classauto__apms__behavior__tree_1_1TreeBuildHandler.html#ac4c4887fcd65b024a89445009cbe15b4)

    Callback for interpreting the build request. Depending on the provided arguments, the build handler should decide whether to accept the request or not. If it accepts it, it's common practice to cache this information for later use inside `buildTree`.

    By default (if this method is not overridden), all requests are accepted and the arguments of the build request are ignored. This typically means that the build handler is able to create the tree regardless of the user input.

- [`TreeBuildHandler::buildTree`](https://robin-mueller.github.io/auto-apms/classauto__apms__behavior__tree_1_1TreeBuildHandler.html#afbe6b3c853a2eabad3ff5c48fb50db3d)

    Callback for actually building the behavior tree. It is **imperative to override** this method. If `setBuildRequest` is overridden, this method is expected to refer to the information provided by the build request and create the tree accordingly. It's the user's responsibility to implement a sensible algorithm which might as well not use the build request to determine which tree is created.

When using AutoAPMS's behavior executor, **`setBuildRequest` is always invoked before `buildTree`** as expected. Which implementation the executor should use can be configured using the `build_handler` parameter. The user selects a build handler by passing the corresponding [resource identity](./common-resources.md#build-handler-identity).

This architecture offers great flexibility because it allows the user to not just define arbitrary build requests, but also arbitrary interpretation rules. The only limitation is that the build request must be serializable (i.e. representable by a string).

## Global Blackboard

::: warning Global doesn't mean system-wide!
With "global" we emphasize that this behavior tree blackboard is available to all subtrees and is not associated with only a single behavior tree. However, each `TreeExecutorNode` holds an independent global blackboard instance.
:::

## Scripting Enums

## Configuration Parameters

Here is a list of all statically declared parameters implemented by `TreeExecutorNode`.

By default, the following values apply:

```yaml
build_handler: auto_apms_behavior_tree::TreeFromResourceBuildHandler
tick_rate: 0.1
allow_other_build_handlers: true
allow_dynamic_scripting_enums: true
allow_dynamic_blackboard: true
build_handler_exclude_packages: '{}'
node_exclude_packages: '{}'
groot2_port: -1.0
state_change_logger: false
```

| Parameter Name | Type | Description |
| :---: | :---: | :--- |
| `build_handler` | `string` | Fully qualified class name of the behavior tree build handler responsible for creating trees if not overridden by the `StartTreeExecutor` action goal. |
| `tick_rate` | `double` | Interval [s] at which the behavior tree is being ticked. |
| `allow_other_build_handlers` | `bool` (Read only) | Option whether to allow dynamic loading/unloading of behavior tree build handler plugins. |
| `allow_dynamic_scripting_enums` | `bool` | Option whether to allow dynamically changing scripting enum parameters. |
| `allow_dynamic_blackboard` | `bool` | Option whether to allow dynamically changing blackboard parameters. |
| `build_handler_exclude_packages` | `string_array` (Read only) | List of package names to exclude when searching for behavior tree build handler plugins. |
| `node_exclude_packages` | `string_array` (Read only) | List of package names to exclude when searching for behavior tree node plugins. |
| `groot2_port` | `int` | Server port for the Groot2 publisher. `-1` means that it won't be created. |
| `state_change_logger` | `bool` | Flag whether to allow the behavior tree state observer to write to rosout. |
