---
order: 10
---
# Tree Executor Parameter Reference

Here is a list of all parameters supported by `TreeExecutorNode`.

By default, the following configuration applies:

```yaml
allow_dynamic_blackboard: true
allow_dynamic_scripting_enums: true
allow_other_build_handlers: true
build_handler: auto_apms_behavior_tree::TreeFromResourceBuildHandler
build_handler_exclude_packages: '{}'
groot2_port: -1.0
node_exclude_packages: '{}'
state_change_logger: false
tick_rate: 0.1
```

## allow_other_build_handlers

Option whether to allow dynamic loading/unloading of tree build handler plugins.

- Type: `bool`
- Default Value: true
- Read only: True

## allow_dynamic_scripting_enums

Option whether to allow dynamically changing scripting enum parameters.

- Type: `bool`
- Default Value: true

## allow_dynamic_blackboard

Option whether to allow dynamically changing blackboard parameters.

- Type: `bool`
- Default Value: true

## node_exclude_packages

List of package names to exclude when searching for tree node plugins.

- Type: `string_array`
- Default Value: {}
- Read only: True

*Constraints:*

- contains no duplicates

## build_handler_exclude_packages

List of package names to exclude when searching for tree build handler plugins.

- Type: `string_array`
- Default Value: {}
- Read only: True

*Constraints:*

- contains no duplicates

## build_handler

Fully qualified class name of the behavior tree build handler responsible for creating trees if not overridden by the StartTreeExecutor action goal.

- Type: `string`
- Default Value: "auto_apms_behavior_tree::TreeFromResourceBuildHandler"

## tick_rate

Interval [s] at which the behavior tree is being ticked.

- Type: `double`
- Default Value: 0.1

*Constraints:*

- greater than or equal to 0

## groot2_port

Server port for the Groot2 publisher. -1 means no Groot2 publisher will be created.

- Type: `int`
- Default Value: -1

*Constraints:*

- greater than or equal to -1

## state_change_logger

Flag whether to write any behavior tree state changes to rosout.

- Type: `bool`
- Default Value: false
