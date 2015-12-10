# RoboCup-Logistics Gazebo Simulation

This repository includes the models, worlds and plugins to simulate the RoboCup Logistics League with Gazebo.

## Gazebo
[Installing Gazebo](http://gazebosim.org)


Clone this repository and then set the following environmental variables by adding them to your ~/.bashrc.
You may have to change the path depending on where you cloned the repository.

```bash
source /usr/share/gazebo/setup.sh
export GAZEBO_RCLL=~/gazebo-rcll
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$GAZEBO_RCLL/plugins/lib/gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_RCLL/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$GAZEBO_RCLL/models/carologistics
```

NOTE:
When you want to start Gazebo as a ROS node you need to add those changes in the /usr/share/gazebo/setup.sh.

Then you can start gazebo from the terminal.

## References
The work is based on [this project](http://www.fawkesrobotics.org/projects/llsf-sim/) where you can find more information and an example how to interface the simulation.


