# ROS2_CONTROL SOLO 12

## Overview

This repository simulates a quadruped robot, [SOLO 12](https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware/blob/master/mechanics/quadruped_robot_12dof_v1/README.md), in the
Gazebo simulator. Furthermore, this repository relies only on the effort interface from ROS2_control.

## Installations
We use ROS 2 Foxy. Here are the dependencies used.
- ros2-control
- ros2-controllers
- ros-foxy-gazebo-ros-pkgs
- ros-foxy-xacro
- ros-foxy-gazebo-ros2-control
- ros-foxy-realtime-tools
- ros-foxy-angles
- odri-control-interface and master-board directories from this [link](https://github.com/open-dynamic-robot-initiative/odri_control_interface). These packages and the ones listed below should go in the `src` folder of your workspace.
    - The instructions for getting those packages are available in the README.md file from the link above. Once you compile and obtain them, you can create a symbolic link to the src folder of your current workspace (e.g., for odri-control-interface, `ln -s path-to-odri-control-interface-package path-to-your-workspace-src-folder`)
- mpi_cmake_modules from [here](https://github.com/machines-in-motion/mpi_cmake_modules)
- pybind11 from [here](https://github.com/pybind/pybind11) 
- eigenpy from [here](https://github.com/stack-of-tasks/eigenpy)
- doxygen

You can also run `rosdep install --from-paths src --ignore-src -r -y` to install everything in one command, but it might miss a dependency (as I might not have listed all dependencies). 
Once the dependencies are installed, build the workspace with `colcon build --symlink-install`. Note that if building the workspace consumes too much memory, you can try instead
`MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential`. Finally, if this is your first time building the workspace, it might throw an error about 
a package not found. If so, source your workspace (i.e., `source install/setup.bash`) and try again.

## Running the simulation

The following commands simulate SOLO in Gazebo. **Note that before running the simulation, 
in the package ros2_description_solo, on line 1031 of the file urdf/solo12_simulation.sdf, please write your absolute path to the file src/ros2_control_solo_bringup/config/solo_gazebo_test_controllers.yaml (in the ros2_control_solo_bringup package)** (as I have not figured out how to dynamically determine the absolute path of a file in an SDF file). 
1. Source ROS2 foxy and source your install file in the workspace
2. `source /usr/share/gazebo/setup.sh` (since some Gazebo environment variables will be overridden)
3. In your workspace directory, `ros2 launch ros2_control_solo_bringup solo_system_effort_only_gazebo.launch.py` launches the Gazebo simulation with SOLO.

## Repository Description

[models](models): stores the world files and model files used by the world files.
- a plugin was added to the default `empty.world` from ROS 2, so the current position and orientation of SOLO could be read from Gazebo.

[ros2_control_solo_bringup](ros2_control_solo_bringup): composed of launch files and yaml files storing parameters. Relevant files are:
- `solo_system_effort_only_gazebo.launch.py` launches Gazebo to simulate the robot.
- In the config folder, `solo_gazebo_test_controllers.yaml` is called by `solo12.urdf.xacro` and `solo12_simulation.sdf` in the ros2_description_solo package. It contains parameters used by ros2_control.

[ros2_description_solo](ros2_description_solo): contains the description and meshes files for SOLO 12.
- the 2 folders in use are meshes and urdf
- In the urdf subdirectory, `solo12_simulation.sdf` is manually generated from `solo12.urdf.xacro`. This SDF file is used to spawn SOLO in Gazebo. The SDF file has contact and friction properties that the xacro/URDF file cannot specify. These parameters were manually added.

## Notes
- package.xml and README.md files for individual packages need to be updated.
