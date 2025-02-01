<!-- markdownlint-disable MD024 MD041 -->  
## Overview

| Registration Name | Class Name | Package |
| :--- | :---: | :---: |
| [AlwaysFailure](#alwaysfailure) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [AlwaysSuccess](#alwayssuccess) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [AsyncFallback](#asyncfallback) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [AsyncSequence](#asyncsequence) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Delay](#delay) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Fallback](#fallback) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [ForceFailure](#forcefailure) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [ForceSuccess](#forcesuccess) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [IfThenElse](#ifthenelse) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Inverter](#inverter) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [KeepRunningUntilFailure](#keeprunninguntilfailure) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [LoopBool](#loopbool) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [LoopDouble](#loopdouble) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [LoopInt](#loopint) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [LoopString](#loopstring) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Parallel](#parallel) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [ParallelAll](#parallelall) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Precondition](#precondition) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [ReactiveFallback](#reactivefallback) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [ReactiveSequence](#reactivesequence) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Repeat](#repeat) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [RetryUntilSuccessful](#retryuntilsuccessful) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [RunOnce](#runonce) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Script](#script) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [ScriptCondition](#scriptcondition) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Sequence](#sequence) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [SequenceWithMemory](#sequencewithmemory) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [SetBlackboard](#setblackboard) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [SkipUnlessUpdated](#skipunlessupdated) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Sleep](#sleep) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [SubTree](#subtree) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Switch2](#switch2) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Switch3](#switch3) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Switch4](#switch4) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Switch5](#switch5) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Switch6](#switch6) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Timeout](#timeout) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [UnsetBlackboard](#unsetblackboard) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [WaitValueUpdate](#waitvalueupdate) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [WasEntryUpdated](#wasentryupdated) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [WhileDoElse](#whiledoelse) | `❌` | auto_apms_behavior_tree (BehaviorTree.CPP) |
| [Error](#error) | `auto_apms_behavior_tree::ThrowException` | auto_apms_behavior_tree |
| [GetParameter](#getparameter) | `auto_apms_behavior_tree::GetParameter` | auto_apms_behavior_tree |
| [GetParameterBool](#getparameterbool) | `auto_apms_behavior_tree::GetParameterBool` | auto_apms_behavior_tree |
| [GetParameterBoolVec](#getparameterboolvec) | `auto_apms_behavior_tree::GetParameterBoolVec` | auto_apms_behavior_tree |
| [GetParameterByteVec](#getparameterbytevec) | `auto_apms_behavior_tree::GetParameterByteVec` | auto_apms_behavior_tree |
| [GetParameterDouble](#getparameterdouble) | `auto_apms_behavior_tree::GetParameterDouble` | auto_apms_behavior_tree |
| [GetParameterDoubleVec](#getparameterdoublevec) | `auto_apms_behavior_tree::GetParameterDoubleVec` | auto_apms_behavior_tree |
| [GetParameterInt](#getparameterint) | `auto_apms_behavior_tree::GetParameterInt` | auto_apms_behavior_tree |
| [GetParameterIntVec](#getparameterintvec) | `auto_apms_behavior_tree::GetParameterIntVec` | auto_apms_behavior_tree |
| [GetParameterString](#getparameterstring) | `auto_apms_behavior_tree::GetParameterString` | auto_apms_behavior_tree |
| [GetParameterStringVec](#getparameterstringvec) | `auto_apms_behavior_tree::GetParameterStringVec` | auto_apms_behavior_tree |
| [HaltExecutor](#haltexecutor) | `auto_apms_behavior_tree::HaltExecutor` | auto_apms_behavior_tree |
| [HasParameter](#hasparameter) | `auto_apms_behavior_tree::HasParameter` | auto_apms_behavior_tree |
| [Logger](#logger) | `auto_apms_behavior_tree::Logger` | auto_apms_behavior_tree |
| [PauseExecutor](#pauseexecutor) | `auto_apms_behavior_tree::PauseExecutor` | auto_apms_behavior_tree |
| [ResumeExecutor](#resumeexecutor) | `auto_apms_behavior_tree::ResumeExecutor` | auto_apms_behavior_tree |
| [SetParameter](#setparameter) | `auto_apms_behavior_tree::SetParameter` | auto_apms_behavior_tree |
| [SetParameterBool](#setparameterbool) | `auto_apms_behavior_tree::SetParameterBool` | auto_apms_behavior_tree |
| [SetParameterBoolVec](#setparameterboolvec) | `auto_apms_behavior_tree::SetParameterBoolVec` | auto_apms_behavior_tree |
| [SetParameterByteVec](#setparameterbytevec) | `auto_apms_behavior_tree::SetParameterByteVec` | auto_apms_behavior_tree |
| [SetParameterDouble](#setparameterdouble) | `auto_apms_behavior_tree::SetParameterDouble` | auto_apms_behavior_tree |
| [SetParameterDoubleVec](#setparameterdoublevec) | `auto_apms_behavior_tree::SetParameterDoubleVec` | auto_apms_behavior_tree |
| [SetParameterInt](#setparameterint) | `auto_apms_behavior_tree::SetParameterInt` | auto_apms_behavior_tree |
| [SetParameterIntVec](#setparameterintvec) | `auto_apms_behavior_tree::SetParameterIntVec` | auto_apms_behavior_tree |
| [SetParameterString](#setparameterstring) | `auto_apms_behavior_tree::SetParameterString` | auto_apms_behavior_tree |
| [SetParameterStringVec](#setparameterstringvec) | `auto_apms_behavior_tree::SetParameterStringVec` | auto_apms_behavior_tree |
| [StartExecutor](#startexecutor) | `auto_apms_behavior_tree::StartExecutor` | auto_apms_behavior_tree |
| [TerminateExecutor](#terminateexecutor) | `auto_apms_behavior_tree::TerminateExecutor` | auto_apms_behavior_tree |
| [GetRobotState](#getrobotstate) | `auto_apms_simulation::GetRobotState` | auto_apms_simulation |
| [IsLocationOccupied](#islocationoccupied) | `auto_apms_simulation::IsLocationOccupied` | auto_apms_simulation |
| [NavigateToLocation](#navigatetolocation) | `auto_apms_simulation::NavigateToLocation` | auto_apms_simulation |
| [PickObject](#pickobject) | `auto_apms_simulation::PickObject` | auto_apms_simulation |
| [PlaceObject](#placeobject) | `auto_apms_simulation::PlaceObject` | auto_apms_simulation |
| [RobotSharesCurrentLocation](#robotsharescurrentlocation) | `auto_apms_simulation::RobotSharesCurrentLocation` | auto_apms_simulation |
| [SetLocationState](#setlocationstate) | `auto_apms_simulation::SetLocationState` | auto_apms_simulation |
| [Arm](#arm) | `auto_apms_px4::ArmAction` | auto_apms_px4 |
| [Disarm](#disarm) | `auto_apms_px4::DisarmAction` | auto_apms_px4 |
| [EnableHold](#enablehold) | `auto_apms_px4::EnableHoldAction` | auto_apms_px4 |
| [GoTo](#goto) | `auto_apms_px4::GoToAction` | auto_apms_px4 |
| [GoToVector](#gotovector) | `auto_apms_px4::GoToAction` | auto_apms_px4 |
| [Land](#land) | `auto_apms_px4::LandAction` | auto_apms_px4 |
| [Mission](#mission) | `auto_apms_px4::MissionAction` | auto_apms_px4 |
| [RTL](#rtl) | `auto_apms_px4::RTLAction` | auto_apms_px4 |
| [ReadGlobalPosition](#readglobalposition) | `auto_apms_px4::ReadGlobalPosition` | auto_apms_px4 |
| [Takeoff](#takeoff) | `auto_apms_px4::TakeoffAction` | auto_apms_px4 |

## auto_apms_behavior_tree (BehaviorTree.CPP)

### AlwaysFailure

**Class:** `❌`

**Node Type:** `Action`

*This node doesn't have any ports.*

### AlwaysSuccess

**Class:** `❌`

**Node Type:** `Action`

*This node doesn't have any ports.*

### AsyncFallback

**Class:** `❌`

**Node Type:** `Control`

*This node doesn't have any ports.*

### AsyncSequence

**Class:** `❌`

**Node Type:** `Control`

*This node doesn't have any ports.*

### Delay

**Class:** `❌`

**Node Type:** `Decorator`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **delay_msec** | `unsigned int` | ❌ | Tick the child after a few milliseconds |

### Fallback

**Class:** `❌`

**Node Type:** `Control`

*This node doesn't have any ports.*

### ForceFailure

**Class:** `❌`

**Node Type:** `Decorator`

*This node doesn't have any ports.*

### ForceSuccess

**Class:** `❌`

**Node Type:** `Decorator`

*This node doesn't have any ports.*

### IfThenElse

**Class:** `❌`

**Node Type:** `Control`

*This node doesn't have any ports.*

### Inverter

**Class:** `❌`

**Node Type:** `Decorator`

*This node doesn't have any ports.*

### KeepRunningUntilFailure

**Class:** `❌`

**Node Type:** `Decorator`

*This node doesn't have any ports.*

### LoopBool

**Class:** `❌`

**Node Type:** `Decorator`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **if_empty** | `BT::NodeStatus` | SUCCESS | Status to return if queue is empty: SUCCESS, FAILURE, SKIPPED |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `bool` | ❌ |  |

#### Bidirectional Ports

| Port Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **queue** | `std::shared_ptr<std::deque<bool, std::allocator<bool> > >` | ❌ |  |

### LoopDouble

**Class:** `❌`

**Node Type:** `Decorator`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **if_empty** | `BT::NodeStatus` | SUCCESS | Status to return if queue is empty: SUCCESS, FAILURE, SKIPPED |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `double` | ❌ |  |

#### Bidirectional Ports

| Port Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **queue** | `std::shared_ptr<std::deque<double, std::allocator<double> > >` | ❌ |  |

### LoopInt

**Class:** `❌`

**Node Type:** `Decorator`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **if_empty** | `BT::NodeStatus` | SUCCESS | Status to return if queue is empty: SUCCESS, FAILURE, SKIPPED |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `int` | ❌ |  |

#### Bidirectional Ports

| Port Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **queue** | `std::shared_ptr<std::deque<int, std::allocator<int> > >` | ❌ |  |

### LoopString

**Class:** `❌`

**Node Type:** `Decorator`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **if_empty** | `BT::NodeStatus` | SUCCESS | Status to return if queue is empty: SUCCESS, FAILURE, SKIPPED |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `std::string` | ❌ |  |

#### Bidirectional Ports

| Port Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **queue** | `std::shared_ptr<std::deque<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > > >` | ❌ |  |

### Parallel

**Class:** `❌`

**Node Type:** `Control`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **failure_count** | `int` | 1 | number of children that need to fail to trigger a FAILURE |
| **success_count** | `int` | -1 | number of children that need to succeed to trigger a SUCCESS |

### ParallelAll

**Class:** `❌`

**Node Type:** `Control`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **max_failures** | `int` | 1 | If the number of children returning FAILURE exceeds this value, ParallelAll returns FAILURE |

### Precondition

**Class:** `❌`

**Node Type:** `Decorator`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **else** | `BT::NodeStatus` | FAILURE | Return status if condition is false |
| **if** | `std::string` | ❌ |  |

### ReactiveFallback

**Class:** `❌`

**Node Type:** `Control`

*This node doesn't have any ports.*

### ReactiveSequence

**Class:** `❌`

**Node Type:** `Control`

*This node doesn't have any ports.*

### Repeat

**Class:** `❌`

**Node Type:** `Decorator`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **num_cycles** | `int` | ❌ | Repeat a successful child up to N times. Use -1 to create an infinite loop. |

### RetryUntilSuccessful

**Class:** `❌`

**Node Type:** `Decorator`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **num_attempts** | `int` | ❌ | Execute again a failing child up to N times. Use -1 to create an infinite loop. |

### RunOnce

**Class:** `❌`

**Node Type:** `Decorator`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **then_skip** | `bool` | true | If true, skip after the first execution, otherwise return the same NodeStatus returned once bu the child. |

### Script

**Class:** `❌`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **code** | `std::string` | ❌ | Piece of code that can be parsed |

### ScriptCondition

**Class:** `❌`

**Node Type:** `Condition`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **code** | `BT::AnyTypeAllowed` | ❌ | Piece of code that can be parsed. Must return false or true |

### Sequence

**Class:** `❌`

**Node Type:** `Control`

*This node doesn't have any ports.*

### SequenceWithMemory

**Class:** `❌`

**Node Type:** `Control`

*This node doesn't have any ports.*

### SetBlackboard

**Class:** `❌`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `BT::AnyTypeAllowed` | ❌ | Value to be written int othe output_key |

#### Bidirectional Ports

| Port Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **output_key** | `BT::AnyTypeAllowed` | ❌ | Name of the blackboard entry where the value should be written |

### SkipUnlessUpdated

**Class:** `❌`

**Node Type:** `Decorator`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **entry** | `BT::Any` | ❌ | Entry to check |

### Sleep

**Class:** `❌`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **msec** | `unsigned int` | ❌ |  |

### SubTree

**Class:** `❌`

**Node Type:** `SubTree`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **_autoremap** | `bool` | false | If true, all the ports with the same name will be remapped |

### Switch2

**Class:** `❌`

**Node Type:** `Control`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **case_2** | `std::string` | ❌ |  |
| **case_1** | `std::string` | ❌ |  |
| **variable** | `std::string` | ❌ |  |

### Switch3

**Class:** `❌`

**Node Type:** `Control`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **case_3** | `std::string` | ❌ |  |
| **case_2** | `std::string` | ❌ |  |
| **case_1** | `std::string` | ❌ |  |
| **variable** | `std::string` | ❌ |  |

### Switch4

**Class:** `❌`

**Node Type:** `Control`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **case_4** | `std::string` | ❌ |  |
| **case_3** | `std::string` | ❌ |  |
| **case_2** | `std::string` | ❌ |  |
| **case_1** | `std::string` | ❌ |  |
| **variable** | `std::string` | ❌ |  |

### Switch5

**Class:** `❌`

**Node Type:** `Control`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **case_5** | `std::string` | ❌ |  |
| **case_4** | `std::string` | ❌ |  |
| **case_3** | `std::string` | ❌ |  |
| **case_2** | `std::string` | ❌ |  |
| **case_1** | `std::string` | ❌ |  |
| **variable** | `std::string` | ❌ |  |

### Switch6

**Class:** `❌`

**Node Type:** `Control`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **case_5** | `std::string` | ❌ |  |
| **case_4** | `std::string` | ❌ |  |
| **case_6** | `std::string` | ❌ |  |
| **case_3** | `std::string` | ❌ |  |
| **case_2** | `std::string` | ❌ |  |
| **case_1** | `std::string` | ❌ |  |
| **variable** | `std::string` | ❌ |  |

### Timeout

**Class:** `❌`

**Node Type:** `Decorator`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **msec** | `unsigned int` | ❌ | After a certain amount of time, halt() the child if it is still running. |

### UnsetBlackboard

**Class:** `❌`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **key** | `std::string` | ❌ | Key of the entry to remove |

### WaitValueUpdate

**Class:** `❌`

**Node Type:** `Decorator`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **entry** | `BT::Any` | ❌ | Entry to check |

### WasEntryUpdated

**Class:** `❌`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **entry** | `BT::Any` | ❌ | Entry to check |

### WhileDoElse

**Class:** `❌`

**Node Type:** `Control`

*This node doesn't have any ports.*

## auto_apms_behavior_tree

### Error

**Class:** `auto_apms_behavior_tree::ThrowException`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **message** | `std::string` | ❌ | Error message. Creates a generic error message if empty. |

### GetParameter

**Class:** `auto_apms_behavior_tree::GetParameter`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to get. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `BT::AnyTypeAllowed` | ❌ | Output port for the parameter's value. |

### GetParameterBool

**Class:** `auto_apms_behavior_tree::GetParameterBool`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to get. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `bool` | ❌ | Output port for the parameter's value. |

### GetParameterBoolVec

**Class:** `auto_apms_behavior_tree::GetParameterBoolVec`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to get. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `std::vector<bool, std::allocator<bool> >` | ❌ | Output port for the parameter's value. |

### GetParameterByteVec

**Class:** `auto_apms_behavior_tree::GetParameterByteVec`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to get. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `std::vector<unsigned char, std::allocator<unsigned char> >` | ❌ | Output port for the parameter's value. |

### GetParameterDouble

**Class:** `auto_apms_behavior_tree::GetParameterDouble`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to get. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `double` | ❌ | Output port for the parameter's value. |

### GetParameterDoubleVec

**Class:** `auto_apms_behavior_tree::GetParameterDoubleVec`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to get. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `std::vector<double, std::allocator<double> >` | ❌ | Output port for the parameter's value. |

### GetParameterInt

**Class:** `auto_apms_behavior_tree::GetParameterInt`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to get. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `long` | ❌ | Output port for the parameter's value. |

### GetParameterIntVec

**Class:** `auto_apms_behavior_tree::GetParameterIntVec`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to get. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `std::vector<long, std::allocator<long> >` | ❌ | Output port for the parameter's value. |

### GetParameterString

**Class:** `auto_apms_behavior_tree::GetParameterString`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to get. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `std::string` | ❌ | Output port for the parameter's value. |

### GetParameterStringVec

**Class:** `auto_apms_behavior_tree::GetParameterStringVec`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to get. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **value** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Output port for the parameter's value. |

### HaltExecutor

**Class:** `auto_apms_behavior_tree::HaltExecutor`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **executor** | `std::string` | ❌ | Name of the executor to command. |

### HasParameter

**Class:** `auto_apms_behavior_tree::HasParameter`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

### Logger

**Class:** `auto_apms_behavior_tree::Logger`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **message** | `BT::AnyTypeAllowed` | ❌ | Message to be logged via rclcpp::Logger. |
| **level** | `std::string` | INFO | Logger level. Must be one of [UNSET, DEBUG, INFO, WARN, ERROR, FATAL] but is not case sensitive. |

### PauseExecutor

**Class:** `auto_apms_behavior_tree::PauseExecutor`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **executor** | `std::string` | ❌ | Name of the executor to command. |

### ResumeExecutor

**Class:** `auto_apms_behavior_tree::ResumeExecutor`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **executor** | `std::string` | ❌ | Name of the executor to command. |

### SetParameter

**Class:** `auto_apms_behavior_tree::SetParameter`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to be set. |
| **value** | `BT::AnyTypeAllowed` | ❌ | Value of the parameter to be set. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

### SetParameterBool

**Class:** `auto_apms_behavior_tree::SetParameterBool`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to be set. |
| **value** | `bool` | ❌ | Value of the parameter to be set. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

### SetParameterBoolVec

**Class:** `auto_apms_behavior_tree::SetParameterBoolVec`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to be set. |
| **value** | `std::vector<bool, std::allocator<bool> >` | ❌ | Value of the parameter to be set. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

### SetParameterByteVec

**Class:** `auto_apms_behavior_tree::SetParameterByteVec`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to be set. |
| **value** | `std::vector<unsigned char, std::allocator<unsigned char> >` | ❌ | Value of the parameter to be set. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

### SetParameterDouble

**Class:** `auto_apms_behavior_tree::SetParameterDouble`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to be set. |
| **value** | `double` | ❌ | Value of the parameter to be set. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

### SetParameterDoubleVec

**Class:** `auto_apms_behavior_tree::SetParameterDoubleVec`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to be set. |
| **value** | `std::vector<double, std::allocator<double> >` | ❌ | Value of the parameter to be set. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

### SetParameterInt

**Class:** `auto_apms_behavior_tree::SetParameterInt`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to be set. |
| **value** | `long` | ❌ | Value of the parameter to be set. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

### SetParameterIntVec

**Class:** `auto_apms_behavior_tree::SetParameterIntVec`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to be set. |
| **value** | `std::vector<long, std::allocator<long> >` | ❌ | Value of the parameter to be set. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

### SetParameterString

**Class:** `auto_apms_behavior_tree::SetParameterString`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to be set. |
| **value** | `std::string` | ❌ | Value of the parameter to be set. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

### SetParameterStringVec

**Class:** `auto_apms_behavior_tree::SetParameterStringVec`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **parameter** | `std::string` | ❌ | Name of the parameter to be set. |
| **value** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Value of the parameter to be set. |
| **node** | `std::string` | ❌ | Name of the targeted ROS 2 node. Leave empty to target this executor's node. |

### StartExecutor

**Class:** `auto_apms_behavior_tree::StartExecutor`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **executor** | `std::string` | ❌ | Name of the executor responsible for building and running the specified behavior tree. |
| **build_request** | `std::string` | ❌ | String passed to the tree build handler defining which tree is to be built. |
| **build_handler** | `std::string` |  | Fully qualified class name of the build handler that is supposed to take care of the request. If empty, use the current one. |
| **root_tree** | `std::string` |  | Name of the root tree. If empty, let the build handler determine the root tree. |
| **node_manifest** | `std::string` |  | YAML/JSON formatted string encoding the name and the registration options for the tree nodes supposed to be loaded before building the tree. |
| **node_overrides** | `std::string` |  | YAML/JSON formatted string encoding the name and the registration options for any tree nodes supposed to override previously loaded ones. |
| **clear_blackboard** | `bool` | true | Boolean flag wether to clear the existing blackboard entries before the execution starts or not. |
| **attach** | `bool` | true | Boolean flag wether to attach to the execution process or start in detached mode. |

### TerminateExecutor

**Class:** `auto_apms_behavior_tree::TerminateExecutor`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **executor** | `std::string` | ❌ | Name of the executor to command. |

## auto_apms_simulation

### GetRobotState

**Class:** `auto_apms_simulation::GetRobotState`

**Node Type:** `Condition`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **robot** | `std::string` | ❌ | Name of the robot. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **location** | `std::string` | {=} | Current location name. |
| **battery** | `double` | {=} | Current battery state [%]. |

### IsLocationOccupied

**Class:** `auto_apms_simulation::IsLocationOccupied`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **target_loc** | `std::string` | ❌ | Name of the location to test for occupancy. |
| **filter_loc** | `std::string` | .* | Regex filter for locations to match against the target location. |
| **filter_robot** | `std::string` | .* | Regex filter for robots to consider. |

### NavigateToLocation

**Class:** `auto_apms_simulation::NavigateToLocation`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **robot** | `std::string` | ❌ | Name of the robot. |
| **target** | `std::string` | ❌ | Name of the target to navigate to (Terms for the knowledge query are separated by whitespace). |

### PickObject

**Class:** `auto_apms_simulation::PickObject`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **robot** | `std::string` | ❌ | Name of the robot. |
| **object** | `std::string` |   | Name of the object to pick. Empty for the nearest object. |

### PlaceObject

**Class:** `auto_apms_simulation::PlaceObject`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **robot** | `std::string` | ❌ | Name of the robot. |

### RobotSharesCurrentLocation

**Class:** `auto_apms_simulation::RobotSharesCurrentLocation`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **robot** | `std::string` | ❌ | Target robot in question to be the first at the given location. |
| **filter_loc** | `std::string` | .* | Regex filter for locations to consider. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **current_loc** | `std::string` | {=} | Current location name. |

### SetLocationState

**Class:** `auto_apms_simulation::SetLocationState`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **open** | `bool` | true | Open/Close the location. |
| **location** | `std::string` | ❌ | Name of the location. |
| **lock** | `bool` | false | Lock/Unlock the location. |

## auto_apms_px4

### Arm

**Class:** `auto_apms_px4::ArmAction`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **wait_until_ready_to_arm** | `bool` | true | Wait for the UAV to be ready for arming. If false and UAV is not ready to arm, will be rejected. |
| **port** | `std::string` | ❌ | Name of the ROS 2 action. |

### Disarm

**Class:** `auto_apms_px4::DisarmAction`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **port** | `std::string` | ❌ | Name of the ROS 2 action. |

### EnableHold

**Class:** `auto_apms_px4::EnableHoldAction`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **port** | `std::string` | ❌ | Name of the ROS 2 action. |

### GoTo

**Class:** `auto_apms_px4::GoToAction`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **lat** | `double` | ❌ | Target latitude |
| **lon** | `double` | ❌ | Target longitude |
| **alt** | `double` | ❌ | Target altitude in meter (AMSL) |
| **port** | `std::string` | ❌ | Name of the ROS 2 action. |

### GoToVector

**Class:** `auto_apms_px4::GoToAction`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **lat** | `double` | ❌ | Target latitude |
| **lon** | `double` | ❌ | Target longitude |
| **alt** | `double` | ❌ | Target altitude in meter (AMSL) |
| **port** | `std::string` | ❌ | Name of the ROS 2 action. |

### Land

**Class:** `auto_apms_px4::LandAction`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **port** | `std::string` | ❌ | Name of the ROS 2 action. |

### Mission

**Class:** `auto_apms_px4::MissionAction`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **do_restart** | `bool` | false | Wether to restart (true) or resume (false) the mission. |
| **port** | `std::string` | ❌ | Name of the ROS 2 action. |

### RTL

**Class:** `auto_apms_px4::RTLAction`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **port** | `std::string` | ❌ | Name of the ROS 2 action. |

### ReadGlobalPosition

**Class:** `auto_apms_px4::ReadGlobalPosition`

**Node Type:** `Condition`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **port** | `std::string` | ❌ | Name of the ROS 2 topic to subscribe to. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **pos_vec** | `Eigen::Matrix<double, 3, 1, 0, 3, 1>` | {pos_vec} | Current global position vector (latitude [°], longitude [°], altitude AMSL [m]) |
| **alt** | `double` | {alt} | Current altitude in meter (AMSL) |
| **lat** | `double` | {lat} | Current latitude in degree [°] |
| **lon** | `double` | {lon} | Current longitude in degree [°] |

### Takeoff

**Class:** `auto_apms_px4::TakeoffAction`

**Node Type:** `Action`

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **alt** | `double` | ❌ | Target takeoff altitude in meter (AMSL) |
| **port** | `std::string` | ❌ | Name of the ROS 2 action. |
