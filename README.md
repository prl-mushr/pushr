# PuSHR: A Multirobot System for Nonprehensile Rearrangement

## Installation

Clone this repo:

```bash
git clone --recurse-submodules https://github.com/prl-mushr/pushr.git
```

Copy each submodule into your catkin workspace, then follow installation instructions for each of the submodules. Installing limit_surface is not required to run the system.

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

## Visualizing with rviz

In a new terminal:

```bash
rviz -d ~/catkin_ws/src/clcbs_ros/rviz/clcbs.rviz
```

Make sure to start rviz before launching `init_clcbs.launch`.
