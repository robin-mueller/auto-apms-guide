---
order: 0
sidebar: ros2behavior
---
# `ros2behavior` CLI Reference

We offer a powerful extension to the ROS 2 command line interface (CLI) by introducing the `ros2 behavior` verb, an ideal supplement to the functionality described by this user guide. It introduces convenient tab completion and allows for quick introspection of installed AutoAPMS resources. It is the recommended way of interacting with everything behavior-related in the workspace.

To install this extension, make sure to build the `auto_apms_ros2behavior` package and source the workspace. Afterwards, you may run

```bash
ros2 behavior -h
```

## **`ros2 behavior`**

This is the entrypoint for all subcommands.

**Usage:**

```bash
ros2 behavior <subcommand> [arguments] [options]
```

**Subcommands:**

- [`list`](#ros2-behavior-list) - List all available behavior resources
- [`show`](#ros2-behavior-show) - Show the content of a behavior resource
- [`run`](#ros2-behavior-run) - Execute a behavior locally
- [`send`](#ros2-behavior-send) - Send a behavior to a running executor
- [`node`](#ros2-behavior-node) - Subcommand for behavior tree node operations

### **`ros2 behavior list`**

List all available behavior resources in the system.

**Usage:**

```bash
ros2 behavior list [options]
```

**Options:**

| Option | Description |
|---|---|
| `-c, --categories [CATEGORIES ...]` | List behavior resources in specified categories. If no category is given, all resources are listed. |
| `--include-internal` | Include behaviors marked as internal (flag). |

**Examples:**

```bash
# List all available behaviors
ros2 behavior list

# List behaviors in specific categories
ros2 behavior list -c navigation control

# Include internal behaviors
ros2 behavior list --include-internal
```

**Output:**
Behaviors are grouped by category and displayed hierarchically:

```text
navigation::
  - goto_waypoint
  - follow_path
control::
  - pid_controller
  - velocity_controller
```

---

### **`ros2 behavior show`**

Display the content/build request of a specific behavior resource.

**Usage:**

```bash
ros2 behavior show <behavior_resource>
```

**Arguments:**

- `behavior_resource` - Identity string of the behavior resource to show

**Examples:**

```bash
# Show a specific behavior
ros2 behavior show navigation::goto_waypoint
```

**Output:**
Displays the build request content for the specified behavior resource.

---

### **`ros2 behavior run`**

Execute a behavior tree locally in a standalone executor.

**Usage:**

```bash
ros2 behavior run [behavior_resource] [options]
```

**Arguments:**

- `behavior_resource` - Identity string of the behavior resource to execute (optional)

**Options:**

| Option | Description |
|---|---|
| `--build-handler <namespace>::<class_name>` | Override the default behavior build handler. |
| `--blackboard [key:=value ...]` | Blackboard variables to pass to the behavior tree. |
| `--tick-rate <float>` | Tick rate for the behavior tree in seconds. |
| `--groot2-port <int>` | Port for Groot2 visualization (disabled by default). |
| `--state-change-logger` | Enable the state change logger (flag). |
| `--logging <level>` | Set logger level (debug, info, warn, error, fatal). |

**Examples:**

```bash
# Run a behavior with default settings
ros2 behavior run navigation::goto_waypoint

# Run with custom blackboard variables
ros2 behavior run navigation::goto_waypoint --blackboard target_x:=10.0 target_y:=5.0

# Run with custom tick rate and Groot2 visualization
ros2 behavior run navigation::goto_waypoint --tick-rate 2.0 --groot2-port 1667

# Run with debug logging and state change logger
ros2 behavior run navigation::goto_waypoint --logging debug --state-change-logger
```

---

### **`ros2 behavior send`**

Send a behavior tree to a running executor and start execution.

**Usage:**

```bash
ros2 behavior send <executor_name> <behavior_resource> [options]
```

**Arguments:**

- `executor_name` - Name of the behavior tree executor to send the tree to
- `behavior_resource` - Identity string of the behavior resource to execute

**Options:**

| Option | Description |
|---|---|
| `--build-handler <namespace>::<class_name>` | Override the default behavior build handler. |
| `--blackboard [key:=value ...]` | Blackboard variables to pass to the behavior tree. |
| `--keep-blackboard` | Do not explicitly clean the blackboard before execution (flag). |
| `--tick-rate <float>` | Tick rate for the behavior tree in seconds (keeps current if omitted). |
| `--groot2-port <int>` | Port for Groot2 visualization (keeps current if omitted). |
| `--state-change-logger <true/false>` | Enable/disable state change logger (keeps current if omitted). |
| `--logging <level>` | Set logger level (keeps current if omitted). |

**Examples:**

```bash
# Send behavior to executor
ros2 behavior send my_executor navigation::goto_waypoint

# Send with blackboard variables
ros2 behavior send my_executor navigation::goto_waypoint --blackboard target_x:=10.0 target_y:=5.0

# Send without clearing existing blackboard
ros2 behavior send my_executor navigation::goto_waypoint --keep-blackboard

# Send with custom settings
ros2 behavior send my_executor navigation::goto_waypoint --tick-rate 1.0 --groot2-port 1667 --state-change-logger true
```

---

## **`ros2 behavior node`**

Subcommand for behavior tree node-related operations. Contains several subcommands itself.

**Usage:**

```bash
ros2 behavior node <subcommand> [options]
```

**Subcommands:**

- [`plugins`](#ros2-behavior-node-plugins) - List all available behavior tree node plugins
- [`manifest`](#ros2-behavior-node-manifest) - Inspect registered behavior tree node manifests
- [`model`](#ros2-behavior-node-model) - Inspect behavior tree node models
- [`call`](#ros2-behavior-node-call) - Execute a single behavior tree node locally

### **`ros2 behavior node plugins`**

List all available behavior tree node plugins grouped by package.

**Usage:**

```bash
ros2 behavior node plugins
```

**Examples:**

```bash
ros2 behavior node plugins
```

**Output:**

```text
Package: auto_apms_behavior_tree
  - ActionNode
  - ConditionNode
  - DecoratorNode
Package: my_custom_package
  - CustomActionNode
  - CustomConditionNode
```

### **`ros2 behavior node manifest`**

Inspect registered behavior tree node manifests and their registered nodes.

**Usage:**

```bash
ros2 behavior node manifest [identity] [node_name]
```

**Arguments:**

- `identity` - Identity string of a node manifest to inspect (optional)
- `node_name` - Registration name for a specific node from the manifest (optional)

**Examples:**

```bash
# List all manifests and their nodes
ros2 behavior node manifest

# Show nodes in a specific manifest
ros2 behavior node manifest navigation::nodes

# Show details for a specific node
ros2 behavior node manifest navigation::nodes goto_waypoint
```

**Output:**

- Without arguments: Lists all manifests and their registered nodes
- With identity: Shows all nodes in that manifest
- With both: Shows YAML configuration for the specific node

### **`ros2 behavior node model`**

Inspect behavior tree node models, showing node types, ports, and detailed information.

**Usage:**

```bash
ros2 behavior node model <manifest> [node_name]
```

**Arguments:**

- `manifest` - Identity string of a node manifest
- `node_name` - Registration name for a specific node (optional)

**Examples:**

```bash
# Show overview of all nodes in a manifest
ros2 behavior node model navigation::nodes

# Show detailed information for a specific node
ros2 behavior node model navigation::nodes goto_waypoint
```

**Output:**

- Without node_name: Shows grouped overview of all nodes by type
- With node_name: Shows detailed information including ports, descriptions, and defaults

### **`ros2 behavior node call`**

Execute a single behavior tree node locally.

**Usage:**

```bash
ros2 behavior node call <manifest> <node_name> [port_values...] [options]
```

**Arguments:**

- `manifest` - Identity string of the node manifest
- `node_name` - Registration name of the node to call
- `port_values` - Port values in `port:=value` format

**Options:**

| Option | Description |
|---|---|
| `--logging <level>` | Set logger level for the executor node. |

**Examples:**

```bash
# Call a node with port values
ros2 behavior node call navigation::nodes goto_waypoint target_x:=10.0 target_y:=5.0

# Call with debug logging
ros2 behavior node call navigation::nodes goto_waypoint target_x:=10.0 --logging debug
```
