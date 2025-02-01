---
order: 20
sidebar: PX4 Skills
---
# PX4 Skill Reference

The package `auto_apms_px4` implements standard skills for PX4 Autopilot:

Class Name | Description
:---: | ---
`auto_apms_px4::ArmDisarmSkill` | Arm respectively disarm the UAS
`auto_apms_px4::EnableHoldSkill` | Enable the standard mode for holding the current position
`auto_apms_px4::GoToSkill` | Fly to the given location
`auto_apms_px4::LandSkill` | Land immediately at the current position
`auto_apms_px4::TakeoffSkill` | Takeoff to a given height
`auto_apms_px4::RTLSkill` | Return to the launch position
`auto_apms_px4::MissionSkill` | Execute the PX4 mission that was uploaded to the UAS

If you want to use those skills, either create you're own launch script and load the nodes using [ROS 2 Composition](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html#) or run

```bash
ros2 launch auto_apms_px4 skills_launch.py
```
