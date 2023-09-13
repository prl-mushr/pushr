# PuSHR: A Multirobot System for Nonprehensile Rearrangement

## Organization

This repository contains four main submodules, each corresponding to a different component of the system.

### limit_surface

limit_surface contains the code for computing limit surfaces and car kinematic constraints for stable pushing. More details can be found in the submodule README.

### mushr_coordination

mushr_coordination contains the code for generating an optimal task assignment from robots to objects. For each number of robots you wish to run the system with, a config file will need to be created as `config/{n}cars.yaml`. Existing examples can be found in the same folder. More details can be found in the submodule README.

### clcbs_ros

clcbs_ros contains the code for generating kinematically feasible plans for all robots, given a task assignment. The code accepts many configuration parameters in the `config/car_params.yaml` file. More details can be found in the submodule README.

### mushr_pixelart_mpc

mushr_pixelart_mpc contains the code for MPC control of each robot, for both sim and real world scenarios. Trajectory sampling and cost weight tuning can be customized in the `config/trajgen/dispersion.yaml` file. More details can be found in the submodule README.

## Installation

Installation has been tested on Ubuntu 18.04.

Install the [MuSHR platform](https://mushr.io/tutorials/quickstart/).

Install various other prerequisites:

```bash
sudo apt install libompl-dev python-scipy python-networkx python-sklearn
pip install future torch
```

Clone this repo into a new catkin workspace:

```bash
mkdir ~/pushr_ws && cd ~/pushr_ws
git clone --recurse-submodules https://github.com/prl-mushr/pushr.git src
catkin_make
```

Now, in every new terminal you open, run the following to source the workspace:

```bash
source ~/pushr_ws/devel/setup.bash
```

Alternatively, clone this repo to a location of your choice, copy each submodule into your catkin workspace, then follow installation instructions for each of the submodules. Installing limit_surface is not required to run the system.

## Running the system in sim

In separate terminals, run the following commands, replacing text in `{}` with the appropriate values.

Terminal 1:

```bash
roscore
```

Terminal 2:

```bash
roslaunch mushr_coordination mushr_coordination.launch cars_file:={num_cars}cars.yaml
```

Terminal 3:

```bash
roslaunch clcbs_ros clcbs_ros.launch
```

Terminal 4:

```bash
roslaunch mushr_pixelart_mpc multi_sim.launch
```

Terminal 5 (the benchmark file should be located in the `mushr_pixelart_mpc/benchmarks` folder):

```bash
roslaunch clcbs_ros init_clcbs.launch benchmark_file:={name of benchmark_file.yaml}
```

Benchmarks 1-6 and 8-9 in the `mushr_pixelart_mpc/benchmarks` folder were used in our evaluation.

## Running the system in the real world

Ensure you have an accurate and reliable pose tracking system for all robots (we used Optitrack motion capture) that can publish poses over ROS.

Launch terminals 1, 2, and 3 as above.

On each car, launch an MPC controller:

```bash
roslaunch mushr_pixelart_mpc real.launch car_name:={car name} car_pose:={car pose topic}
```

Launch terminal 5 as above.

Note that the MPC might need additional tuning to work well in the real world. Parameters can be tuned in the `config/trajgen/dispersion.yaml` file.

## Visualizing with rviz

In a new terminal:

```bash
rviz -d ~/catkin_ws/src/clcbs_ros/rviz/clcbs.rviz
```

Make sure to start rviz before launching `init_clcbs.launch`.
