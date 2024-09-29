---
order: 10
---

# Simulation
For developing missions for UASs using [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot), it is useful to set up the simulation environment provided by the software stack. For that, you need to download the resources introduced in the following sections.

> [!TIP]
> We also recommend to check out the VS Code tasks related to launching a simulation which are available with the [VS Code workspace template](./additional-software#visual-studio-code-workspace).

## PX4 Autopilot
The simulation module for PX4 comes packaged with the source code. AutoAPMS requires at least [v1.15.0](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.15.0).
```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Checkout specific version/branch and update submodules
PX4_VERSION=v1.15.0
(cd ./PX4-Autopilot && git checkout $PX4_VERSION && make submodulesclean)

# Install PX4 Autopilot dependencies on current system
sh ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
If you ever find yourself switching revisions, you have to follow the steps below to cleanly update the PX4 source code:
```sh
cd ./PX4-Autopilot
make clean
make distclean
git checkout v1.15.0 # Switch version/tag/revision here
make submodulesclean # Make sure to update the submodules
```
Visit the [Gaezebo Simulation Guide](https://docs.px4.io/main/en/sim_gazebo_gz/) in the official documentation for learning how to launch a simulation environment for specific vehicles or introduce customized worlds.

## Micro XRCE-DDS Agent
The Micro XRCE-DDS Agent is required to connect to the PX4 middleware and allow ROS 2 to interprete internal PX4 messages as ROS topics. The middleware is described in detail by the official documentation on the [PX4-ROS 2/DDS Bridge](https://docs.px4.io/main/en/middleware/uxrce_dds.html). All you need to know for now is that you need to [install the Micro XRCE-DDS Agent](https://docs.px4.io/main/en/middleware/uxrce_dds.html#micro-xrce-dds-agent-installation). We recommend adding the Git repository to your ROS workspace. Append the following snippet to your `.repos` file and fetch the package just like any other dependency using `vcstool`.
```yaml
microxrcedds_agent:
    type: git
    url: https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    version: v2.4.2
```
> [!IMPORTANT]
> ROS 2 Humble doesn't support newer versions than 2.4.2 due to the fact that it only comes with Fast CDR 1.0.x.

As soon as the source code is available in your ROS workspace, you can build the package with `colcon`:
```sh
colcon build --packages-select microxrcedds_agent
```
Afterwards, you should be able to connect to the Micro XRCE-DDS client running on the simulator:
```sh
MicroXRCEAgent udp4 -p 8888
```

## QGroundControl
QGroundControl (QGC) provides a powerful GUI that allows you to supervise and intervene with the current mission. It's really useful as a manual control panel when developing with AutoAPMS. Therefore, you should download the latest version of [QGroundControl](https://github.com/mavlink/qgroundcontrol/releases) and follow the [installation guide](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu).

> [!NOTE]
> As of now, the latest release of QGC does not yet show the names of custom ROS modes in the mode list.