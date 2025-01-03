# VisFormationPlanner
A visibility-aware formation planning framework that can deal with visibility of the target.

<p align="center">
  <img src="https://github.com/HyPAIR/VisFormationPlanner/blob/main/case1.gif" alt="case1" width="600">
</p>

<p align="center">
  <img src="https://github.com/HyPAIR/VisFormationPlanner/blob/main/case2.gif" alt="case2" width="600">
</p>

## Features

 - A heuristic exploration method which efficiently evaluates a sequence of formation configuration.
 
 - An iterative motion planning framework for finding locally visibility-optimal collision-free formation trajectories.

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
