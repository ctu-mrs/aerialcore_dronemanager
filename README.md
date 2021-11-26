# AerialCore drone manager

This branch contains the [Sequential Function Chart](https://en.wikipedia.org/wiki/Sequential_function_chart) designed for the bird diverter installation experiments.

Compared to what is present in the master at commit @e6fcc91, the following features have been added.

* The _drone manager replanner UAVx_ [drone_manager_replanner_uav1.py:timerMain()](./scripts/drone_manager_replanner_uav1.py) controls drone operations in the experiments with the replanner. In other words, the drones take-off and the mission starts. One of drones reaches a refilling stations and unexpectedly stops its motion. In response to this, a backup drone hosted on another refilling station takes care of its job and plans the trajectory to complete the mission successfully.
* The _drone manager UAVx_ [drone_manager_replanner_uav1.py:timerMain()](./scripts/drone_manager_uav1.py) control the drone operations to fullfill the bird diverter installation tasks. Disturbances and unexpected events are not taken into consideration.
* example tmux session for simulation in [here](./tmux)
* the _world frame_ name and path to the trajectory file is specified in the [launch file](./launch/drone_manager.launch)
* check the [config file](./config/drone_manager.yaml) for additional options

**NOTE** Word file, trajectories, and drone manager files have been customized for the experiments. They are not the one reported in this repository.

## TODO

**This is just a _template_, i.e., a _minimalistic implementaion_. I don't want to see this running on a real drone as it is! Safety conditions and overall sanity checks should be implemented.**
