---
order: 10
---
# Common Resources
<!-- markdownlint-disable MD024 -->  

This page provides the most relevant information for all behavior related resources required when using AutoAPMS. To manage these resources, we incorporate one of ROS's most important core packages: [`ament_index`](https://github.com/ament/ament_index?tab=readme-ov-file). It offers a concept for distributing non-compiled files within a ROS 2 workspace. It's a powerful tool that we heavily rely on to automate troublesome filesystem related processes.

*High-level:*

- [Behavior Trees](#behavior-trees)
- [Mission Configurations](#mission-configurations)

*Low-level:*

- [Behavior Tree Build Handlers](#behavior-tree-build-handlers)
- [Behavior Tree Node Manifests](#behavior-tree-node-manifests)
- [Behavior Tree Node Models](#behavior-tree-node-models)

## Behavior Trees

Probably the most important non-compiled resource when using AutoAPMS is the behavior tree XML file. The schema of such a file is defined [here](https://www.behaviortree.dev/docs/learn-the-basics/xml_format)
and looks similar to this:

```xml
<root BTCPP_format="4" main_tree_to_execute="RootTree">
  <!-- Each TreeDocument may have zero or more behavior tree elements -->
  <BehaviorTree ID="RootTree">
    <!-- Each behavior tree element has exactly one child. This child may again have zero or more children -->
  </BehaviorTree>
  <BehaviorTree ID="AnotherTree">
    <!-- ... -->
  </BehaviorTree>
</root>
```

Behavior tree resources are created using [`auto_apms_behavior_tree_declare_trees`](../../reference/cmake.md#declare-trees).

::: info Learn more 🎓
Visit the tutorial [Building Behavior Trees: Graphical Approach](../tutorials/building-behavior-trees.md#graphical-approach) for more information about how behavior tree resources are created.
:::

### Resource Identity Format {#tree-identity}

This is the full signature of a behavior tree's resource identity:

- `<package_name>::<tree_file_stem>::<tree_name>`

| Token Name | Description |
| :---: | :--- |
| `<package_name>` | Name of the ROS 2 package that registers the resource. |
| `<tree_file_stem>` | Name of the behavior tree XML file (without extension) that contains the data. |
| `<tree_name>` | Name of a specific tree inside `<tree_file_stem>.xml` that is supposed to be the root tree. |

#### Example

Given the following configuration of a package called `my_package`:

```cmake [CMakeLists.txt]
project(my_package)
# ...
auto_apms_behavior_tree_declare_trees(
    "config/my_behavior_tree.xml"
    "config/another_behavior_tree.xml"
    # ...
)
```

The available node manifest resources can be queried like this:

```cpp [source.cpp]
#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

using TreeResource = auto_apms_behavior_tree::core::TreeResource;
using TreeResourceIdentity = auto_apms_behavior_tree::core::TreeResourceIdentity;
using TreeDocument = auto_apms_behavior_tree::core::TreeDocument;
 
// For example, use the fully qualified tree resource identity signature
const std::string identity_string = "my_package::my_behavior_tree::MyTreeName";
 
// You may use the proxy class for a tree resource identity
TreeResourceIdentity identity(identity_string);
TreeResource resource(identity);
 
// Or instantiate the resource object directly using the corresponding identity
TreeResource resource(identity_string);
 
// The resource object may for example be used with TreeDocument
TreeDocument doc;
doc.mergeResource(resource);  // Add "MyTreeName" to the document
 
// This also works, so creating a resource object is not strictly necessary
doc.mergeResource(identity_string)
 
// The simplest approach is this
doc.mergeResource("my_package::my_behavior_tree::MyTreeName");
```

## Mission Configurations

::: info Learn more 🎓
Visit the tutorial [Executing Missions: Configuring a Mission](../tutorials/executing-missions.md#configuring-a-mission) for more information about how mission configurations are created.
:::

### Resource Identity Format {#mission-config-identity}

This is the full signature of a mission configuration's resource identity:

- `<package_name>::<config_file_stem>`

| Token Name | Description |
| :---: | :--- |
| `<package_name>` | Name of the ROS 2 package that registers the resource. |
| `<config_file_stem>` | Name of the mission configuration YAML file (without extension) that contains the data. |

## Behavior Tree Build Handlers

Behavior tree build handlers allow to integrate customized algorithms for creating behavior trees with AutoAPMS's [behavior tree executor](./behavior-tree-executor.md). They are used to define rules for answering the "build requests" formulated when [deploying a behavior](../tutorials/deploying-a-behavior.md).

AutoAPMS provides the following commonly used build handlers out of the box:

- `auto_apms_behavior_tree::TreeFromResourceBuildHandler`

    **Loads a specific behavior tree from the workspace's resource index.**

    This build handler parses the installed [behavior tree resources](#behavior-trees) and builds one of the trees that have previously been declared by the user. The **build request is expected to be the resource identity** of the desired behavior tree.

- `auto_apms_behavior_tree::TreeFromStringBuildHandler`

    **Creates the tree by parsing the provided XML string.**

    The user must encode the behavior tree to be created using the [XML format](https://www.behaviortree.dev/docs/learn-the-basics/xml_format). The **build request is expected to be the XML string**. This build handler additionally requires the user to specify a node manifest for all the nodes used inside the tree.

::: info Learn more 🎓
Visit the tutorial [Building Behavior Trees: Using `TreeBuildHandler`](../tutorials/building-behavior-trees.md#using-treebuildhandler) to learn how to implement your own abstractions.
:::

### Resource Identity Format {#build-handler-identity}

This is the full signature of a build handler's resource identity:

- `<namespace>::<class_name>`

This is also referred to as the "fully qualified class name". We use the same approach for referring to specific build handler plugin as [ROS 2 Composition](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html) does for `rclcpp::Node` components.

| Token Name | Description |
| :---: | :--- |
| `<namespace>` | Full C++ namespace that the class `<class_name>` can be found in. It can be flat `foo` or arbitrarily nested `foo::bar` (individual names must be separated by `::`). |
| `<class_name>` | Name you specified when declaring the build handler class using the `class` keyword in C++. |

## Behavior Tree Node Manifests

Behavior tree node manifests are YAML files used for specifying the [`NodeRegistrationOptions`](https://robin-mueller.github.io/auto-apms/structauto__apms__behavior__tree_1_1core_1_1NodeRegistrationOptions.html) required when loading node plugins implemented by the user. Programmatically, a node manifest is represented by [`NodeManifest`](https://robin-mueller.github.io/auto-apms/classauto__apms__behavior__tree_1_1core_1_1NodeManifest.html). The format of the corresponding YAML file must be as follows:

```yaml [node_manifest.yaml]
MyCustomNodeRegistrationName:  # Registration name of the node
  class_name: my_namespace::MyCustomNodeClass
  port: (input:port)
  wait_timeout: 3
  request_timeout: 2
  allow_unreachable: false
  logger_level: INFO

AnotherCoolNodeName:
  class_name: another_namespace::AnotherCoolNodeClass

# ...
```

| Parameter Name | Required/Optional | Interpreted Type | Description |
| :--- | :---: | :---: | :--- |
| `class_name` | Required | `std::string` | Fully qualified class name (includes all namespace levels separated by `::`) of the behavior tree node plugin that implements the functionality accessible to the behavior tree using the given registration name. We use the same approach for referring to specific behavior tree nodes as [ROS 2 Composition](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html) does for `rclcpp::Node` components. |
| `port` | Optional | `std::string` | *Only relevant for ROS 2 interface nodes.* Name of the ROS 2 action/service/topic to communication with. |
| `wait_timeout` | Optional | `double` | *Only relevant for ROS 2 interface nodes.* Period [s] (measured from tree construction) after the server is considered unreachable. |
| `request_timeout` | Optional | `double` | *Only relevant for ROS 2 interface nodes.* Period [s] (measured from sending a goal request) after the node aborts waiting for a server response. |
| `allow_unreachable` | Optional | `bool` | *Only relevant for ROS 2 interface nodes.* Flag whether to tolerate if the action/service is unreachable when trying to create the client. If set to `true`, a warning is logged. Otherwise, an exception is raised. |
| `logger_level` | Optional | `std::string` | *Only relevant for ROS 2 interface nodes.* Minimum severity level enabled for logging using the ROS 2 Logger API. |

You should understand that registration options are directly associated with a particular registration name (key of the root YAML map).

You may include an arbitrary number of nodes in a node manifest. It's also possible to use the same implementation (specify the same node class name) for different registration names. However, all node registration names within a node manifest must be unique (the YAML specification doesn't allow duplicate map keys anyways).

Only the `class_name` option is required. The example above shows the default values that will be used if any optional parameters are omitted. The node classes referred to by `class_name` must have been declared using [`auto_apms_behavior_tree_declare_nodes`](../../reference/cmake.md#auto-apms-behavior-tree-declare-nodes) before the node manifest is parsed (specifying the node manifest in the same macro call is ok).

::: info Learn more 🎓
Visit the tutorial [Implementing Behavior Tree Nodes: Adding Node Manifests](../tutorials/implementing-behavior-tree-nodes.md#adding-node-manifests) for more information about how to register a node manifest with the resource index.
:::

### Resource Identity Format {#node-manifest-identity}

This is the full signature of a node manifest's resource identity:

- `<package_name>::<metadata_id>`

| Token Name | Description |
| :---: | :--- |
| `<package_name>` | Name of the ROS 2 package that registers the resource. |
| `<metadata_id>` | ID of the metadata that is automatically generated by CMake when specifying the `NODE_MANIFEST` keyword argument of `auto_apms_behavior_tree_declare_nodes` or `auto_apms_behavior_tree_declare_trees`. |

| How is `<metadata_id>` determined? | Rule |
| :---: | :--- |
| [`auto_apms_behavior_tree_declare_nodes`](../../reference/cmake.md#auto-apms-behavior-tree-declare-nodes) | The same as the associated shared library target (first positional argument `target`). |
| [`auto_apms_behavior_tree_declare_trees`](../../reference/cmake.md#auto-apms-behavior-tree-declare-trees) | File stem(s) of the behavior tree XML file(s) under the `paths` argument. |

#### Example

Given the following configuration of a package called `my_package`:

```cmake [CMakeLists.txt]
project(my_package)
# ...
auto_apms_behavior_tree_declare_nodes(behavior_tree_nodes
    # ...
    NODE_MANIFEST
    "config/my_node_manifest.yaml"
)
# ...
auto_apms_behavior_tree_declare_trees(
    "config/my_behavior_tree.xml"
    NODE_MANIFEST
    "config/my_node_manifest.yaml"
)
```

The available node manifest resources can be queried like this:

::: code-group

```cmake [CMakeLists.txt]
auto_apms_behavior_tree_declare_nodes(another_target
    # ...
    NODE_MANIFEST
    "my_package::behavior_tree_nodes"
    "my_package::my_behavior_tree"
)
# OR
auto_apms_behavior_tree_declare_trees(
    # ...
    NODE_MANIFEST
    "my_package::behavior_tree_nodes"
    "my_package::my_behavior_tree"
)
```

```cpp [source.cpp]
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"

using namespace auto_apms_behavior_tree::core;

NodeManifest manifest1 = NodeManifest::fromResourceIdentity("my_package::behavior_tree_nodes");
NodeManifest manifest2 = NodeManifest::fromResourceIdentity("my_package::my_behavior_tree");
```

:::

## Behavior Tree Node Models

- Node model files for Groot2, how to use them
- Node model classes for the builder API
- Pitfalls when trying to load a model file (if possible, should be generated when declaring nodes instead of trees)

### Node Model XML File

### C++ Node Model Header

::: info Learn more 🎓
Check out the usage example [Using `TreeDocument`: Inserting custom nodes](../tutorials/building-behavior-trees.md#usage-examples) for more information about how node models are created and may be used in a C++ source file.
:::
