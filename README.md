# PuSHR: A Multirobot System for Nonprehensile Rearrangement

![](https://github.com/prl-mushr/pushr/blob/master/mushr.gif)

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

The installation has been tested on Ubuntu 18.04. For simulation-only testing and reproducing experiments, we recommend that you use the virtual machine provided for the MuSHR platform. This work is known to have compilation issues on Ubuntu 20.04 (ROS Noetic) due to OMPL compatibility issues.

Install the [MuSHR platform](https://mushr.io/tutorials/quickstart/) (this tutorial also specifies where to get the VM image).

Install various other prerequisites:

```bash
sudo apt install libompl-dev python-scipy python-networkx python-sklearn
pip install future torch
```

Clone this repo into a new catkin workspace. If you are using the MuSHR's VM image, `catkin_ws` need not be created, as you can use the already existing `catkin_ws`. We provide these instructions which create a separate directory for completeness.

```bash
mkdir ~/catkin_ws && cd ~/catkin_ws
git clone --recurse-submodules https://github.com/prl-mushr/pushr.git src
catkin_make
```

The default steering angle limits in the MuSHR stack are lower than what we use in our work. To increase the limits for the real car, change the servo limits in `~/catkin_ws/src/mushr/mushr_base/vesc/vesc_main/config/racecar-uw-nano/vesc.yaml` to 0.0 and 1.0:

```yaml
  servo_min: 0.0
  servo_max: 1.0
```

To increase the limits for the simulated experiments, change the servo limits in `~/catkin_ws/src/mushr_sim/config/vesc.yaml` to 0.0 and 1.0:

```yaml
  servo_min: 0.0
  servo_max: 1.0
```

Now, in every new terminal you open, run the following to source the workspace (only necessary if not using the VM):

```bash
source ~/catkin_ws/devel/setup.bash
```

Alternatively, clone this repo to a location of your choice, copy each submodule into your catkin workspace, then follow installation instructions for each of the submodules.

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

Ensure you have an accurate and reliable pose tracking system for all robots (we used Optitrack motion capture) that can publish poses over ROS. The system expects poses to published to topics named in the format `/{car name}{car pose topic}`, e.g. for a topic `/car30/car_pose`, `car name` would be `car30` and `car pose topic` would be `/car_pose`.

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
