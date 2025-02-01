---
order: 0
sidebar: CMake
---
# CMake Reference
<!-- markdownlint-disable MD024 -->  

AutoAPMS provides the following essential CMake macros that are supposed to be used in the CMakeLists.txt of a specific ROS 2 package.

## `auto_apms_behavior_tree_declare_nodes` {#declare-nodes}

**Add behavior tree node plugins to the resource index.**

This macro must be called to make behavior tree node plugins available at runtime and configure their registration callbacks with the behavior tree factory. Optionally, a corresponding node model header is generated. This header facilitates integrating the specified nodes when building behavior trees using [`TreeDocument`](https://robin-mueller.github.io/auto-apms/classauto__apms__behavior__tree_1_1core_1_1TreeDocument.html).

### Signature

```cmake
auto_apms_behavior_tree_declare_nodes(<target> <class_names>...
    [NODE_MANIFEST node_manifest1 [node_manifest2 ...]]
    [NODE_MODEL_HEADER_TARGET header_target]
)
```

::: warning
All classes passed to `class_names` must also be made discoverable using the C++ macro [`AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE`](https://robin-mueller.github.io/auto-apms/group__auto__apms__behavior__tree.html#ga5d6115d73fc702c19bd6d63860dc2131).

This approach was inspired by [ROS 2 Composition](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html).
:::

| Argument | Required/Optional | Description |
| :--- | :---: | :--- |
| `target` | Required (Positional) | Shared library target implementing the behavior tree nodes declared under `class_names`. |
| `class_names` | Required (Positional) | The unique, fully qualified names of behavior tree node classes (fundamental base must be `BT::TreeNode`) being declared with this macro call and associated with the shared library target. |
| `NODE_MANIFEST` | Optional (Multi-Value-Keyword) | One or more relative paths (relative to `CMAKE_CURRENT_SOURCE_DIR`) or existing resource identities of node manifests. Multiple file paths are concatenated to a single one. |
| `NODE_MODEL_HEADER_TARGET` | Optional (Single-Value-Keyword) | Name of a single shared library target. If specified, generate a C++ header that defines model classes for all behavior tree nodes specified inside the node manifest files provided under `NODE_MANIFEST` and add it to the includes of the given target. |

### Example

```cmake
find_package(auto_apms_behavior_tree REQUIRED)
# ...
auto_apms_behavior_tree_declare_nodes(my_target
    "my_namespace::MyCustomNodeFoo"
    "my_namespace::MyCustomNodeBar"
    NODE_MANIFEST
    "path/to/my_node_manifest.yaml"     # Relative file path
    "other_package::other_metadata_id"  # Resource identity
    NODE_MODEL_HEADER_TARGET
    other_target
)
```

::: info Learn more 🎓
Visit the tutorial [Implementing Custom Behavior Tree Nodes](../usage/tutorials/implementing-behavior-tree-nodes.md) for more information.
:::

## `auto_apms_behavior_tree_declare_trees` {#declare-trees}

**Make behavior trees available to the ROS 2 workspace by adding the corresponding behavior tree XML and node manifest YAML files to the resource index.**

Among other things, this macro configures a behavior tree specific "marker file" that contains essential information for building behavior trees. Read [this guide](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html#the-ament-index-explained) to learn more about the ament resource index.

It allows the user to refer to one of the declared behavior trees using a [resource identity](../usage/concepts/common-resources.md#tree-identity)

### Signature

```cmake
auto_apms_behavior_tree_declare_trees(<paths>...
    [NODE_MANIFEST node_manifest1 [node_manifest2 ...]]
)
```

| Argument | Required/Optional | Description |
| :--- | :---: | :--- |
| `paths` | Required (Positional) | Behavior tree XML files to be added to this package's resources. The user must pass at least one. |
| `NODE_MANIFEST` | Optional (Multi-Value-Keyword) | One or more relative paths (relative to `CMAKE_CURRENT_SOURCE_DIR`) or existing resource identities of node manifests. If specified, behavior tree nodes associated with this manifest can be loaded automatically and are available for every tree under `paths`. |

### Example

```cmake
find_package(auto_apms_behavior_tree REQUIRED)
# ...
auto_apms_behavior_tree_declare_trees(
    "path/to/my_behavior_tree.xml"
    "path/to/another_behavior_tree.xml"
    NODE_MANIFEST
    "path/to/my_node_manifest.yaml"     # Relative file path
    "other_package::other_metadata_id"  # Resource identity
)
```

::: info Learn more 🎓
Visit the tutorial [Building Behavior Trees: Graphical Approach](../usage/tutorials/building-behavior-trees.md#graphical-approach) for more information.
:::

## `auto_apms_behavior_tree_declare_build_handlers` {#declare-build-handlers}

**Add behavior tree build handler plugins to the resource index.**

This macro must be called to make behavior tree build handlers available at runtime. They may be loaded using `TreeBuildHandlerLoader` (a subclass of `pluginlib::ClassLoader`).

### Signature

```cmake
auto_apms_behavior_tree_declare_build_handlers(<target> <class_names>...)
```

::: warning
All classes passed to `class_names` must also be made discoverable using the C++ macro [`AUTO_APMS_BEHAVIOR_TREE_DECLARE_BUILD_HANDLER`](https://robin-mueller.github.io/auto-apms/group__auto__apms__behavior__tree.html#ga45fa41d82d2b212962433e6653b2e0c9).
:::

| Argument | Required/Optional | Description |
| :--- | :---: | :--- |
| `target` | Required (Positional) | Shared library target implementing the behavior tree build handlers declared under `class_names`. |
| `class_names` | Optional (Multi-Value-Keyword) | The unique, fully qualified names of classes inheriting from `TreeBuildHandler` being declared with this macro call and associated with the shared library target. |

### Example

```cmake
find_package(auto_apms_behavior_tree REQUIRED)
# ...
auto_apms_behavior_tree_declare_build_handlers(my_target
    "my_namespace::MyCustomBuildHandlerFoo"
    "my_namespace::MyCustomBuildHandlerBar"
)
```

::: info Learn more 🎓
Visit the tutorial [Building Behavior Trees: Using `TreeBuildHandler`](../usage/tutorials/building-behavior-trees.md#using-treebuildhandler) for more information.
:::

## `auto_apms_mission_register_missions` {#register-missions}

**Register mission configuration files with the resource index.**

This allows the user to refer to one of the registered mission configuration file using a [resource identity](../usage/concepts/common-resources.md#mission-config-identity)

### Signature

```cmake
auto_apms_mission_register_missions(<paths>...)
```

| Argument | Required/Optional | Description |
| :--- | :---: | :--- |
| `paths` | Required (Positional) | Mission configuration YAML files to be added to the resource index. The user must pass at least one. |

### Example

```cmake
find_package(auto_apms_mission REQUIRED)
# ...
auto_apms_mission_register_missions(
    "path/to/my_mission_config.yaml"
    "path/to/another_mission_config.yaml"
)
```

::: info Learn more 🎓
Visit the tutorial [Executing Missions](../usage/tutorials/executing-missions.md) for more information.
:::
