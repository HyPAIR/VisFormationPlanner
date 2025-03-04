# VisFormationPlanner
The source code for the paper "Robots Calling the Shots: Using Multiple Ground Robots for Autonomous Tracking in Cluttered Environments".
<p align="center">
  <img src="https://github.com/HyPAIR/VisFormationPlanner/blob/main/documents/over_view.png" alt="formation_planning" width="729" height=538.5">
</p>

## Features

 - A novel sequential greedy viewpoint planning approach to generate collision-free paths that optimize camera coverage and minimize occlusions.
 
 - A trajectory optimization method that refines the initial planned paths into smooth, kinodynamically feasible trajectories while considering the motion constraints of gimbal-mounted cameras.

## Requirements

 - ROS Noetic or later
 - Ubuntu 20.04 or later
 - You'll also need a license for Gurobi

## Installation

1. Create a new workspace:

```shell
$ mkdir -p ~/visFormationPlanner/src
$ cd ~/visFormationPlanner/src
$ catkin_init_workspace
```

2. Clone the package into the workspace:

```shell
$ git clone git@github.com:HyPAIR/visFormationPlanner.git
```

3. Install dependencies:
```shell
$ rosdep install visFormationPlanner
```

4. Build the workspace:

```shell
$ cd ~/visFormationPlanner
$ catkin_make
```
```shell
$ roslaunch vis_formation_planner heterogeneous_triangle.launch
$ roslaunch vis_formation_planner write_obs_to_world.launch
$ roslaunch vis_formation_planner vis_test.launch
$ roslaunch vis_formation_planner animation_demo.launch
```
