---
order: 10
---
# Installation

## Hardware Compatibility

> [!IMPORTANT]
> AutoAPMS is currenly only available on **Linux** based systems.

| ROS 2 Version |  |
| :-------------: | :-----------: |
| [Humble Hawksbill](https://docs.ros.org/en/humble/index.html) | [![Build and Test](https://github.com/robin-mueller/auto-apms/actions/workflows/build-and-test.yaml/badge.svg)](https://github.com/robin-mueller/auto-apms/actions/workflows/build-and-test.yaml) |

## Setting up the Workspace

AutoAPMS is intended to be built using [`colcon`](https://colcon.readthedocs.io/en/released/user/quick-start.html) just like any other ROS 2 package. Feel encouraged to check out the advantages of using the [recommended VS Code template](./additional-software#visual-studio-code-workspace) for developers which facilitates setting up an AutoAPMS/PX4 development workspace.

To create your first AutoAPMS workspace, follow this step-by-step guide:

> [!NOTE]
> The following assumes that you already installed [ROS 2 (Humble Hawksbill)](https://docs.ros.org/en/humble/Installation.html) `base` + `dev-tools` and created a workspace with a structure similar to the one created in the [official tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

1. Clone the source code to the `src` directory of your workspace:

    ```sh
    # Assuming the current directory is your ROS workspace
    git clone https://github.com/robin-mueller/auto-apms.git src
    ```

1. Install all required ROS packages via `rosdep`:

    ```sh
    rosdep update
    rosdep install --from-paths "src" --ignore-src -y
    ```

1. Some dependencies are not available via `rosdep`, so we need to manually clone the repositories specified in [dependencies.repos](https://github.com/robin-mueller/auto-apms/blob/master/dependencies.repos) (e.g. using [vcstool](https://github.com/dirk-thomas/vcstool)):

    ```sh
    vcs import < "src/auto-apms/dependencies.repos" "src" --recursive
    ```

1. You should now be able to build the `auto_apms` package and all its dependencies:

    ```sh
    colcon build --packages-up-to auto_apms
    ```
