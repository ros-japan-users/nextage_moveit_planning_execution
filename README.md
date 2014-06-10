nextage_moveit_planning_execution
=================================

## Install the dependency packages

```console
$ sudo aptitude update
$ sudo aptitude install ros-%ROS_DISTRO%-rtmros-nextage ros-%ROS_DISTRO%-moveit-ros-visualization ros-%ROS_DISTRO%-moveit-planners-ompl
```

```console
$ sudo aptitude ros-%ROS_DISTRO%-hironx-moveit-config 
```

## Usage

```console
$ rtmlaunch nextage_ros_bridge nextage_ros_bridge_simulation.launch
$ roslaunch nextage_moveit_planning_execution planning_execution.launch
$ rosrun nextage_moveit_planning_execution moveit_command_sender.py
```

![](http://daikimaekawa.github.io/images/moveit/command_sender_plan3.jpg)

## License

Copyright (c) 2014, Daiki Maekawa. (MIT License)

See LICENSE for more info.
