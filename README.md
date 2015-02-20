# RoboCup-Logistics Gazebo Simulation

This repository includes the models, worlds and plugins to simulate the RoboCup Logistics League with Gazebo.

## Gazebo
[Installing Gazebo](http://gazebosim.org)


Clone this repository and then set the path to this database by editing the file `/usr/share/gazebo/setup.sh`.

Add the following entries at the bottom of the file. Change the first line to the correct path of the repository:

```bash
export GAZEBO_RCLL=/home/user/gazebo-rcll
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$GAZEBO_RCLL/worlds
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$GAZEBO_RCLL/plugins/lib/gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$GAZEBO_RCLL/models
# To include team specific models, also add for example this line:
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$GAZEBO_RCLL/models/bbu
```

Make sure that the setup.sh is sourced in your .bashrc:
`source /usr/share/gazebo/setup.sh`

NOTE:
If you add those entries only to your .bashrc file, the plugins will not be found when gazebo is started as a ROS node.

Then you can start gazebo from the terminal.

## References
The work is based on [this project](http://www.fawkesrobotics.org/projects/llsf-sim/) where you can find more information and an example how to interface the simulation.


