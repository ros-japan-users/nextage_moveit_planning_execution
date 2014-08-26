nextage_moveit_planning_execution
=================================

## Install the dependency packages

```sh
$ rosdep install nextage_moveit_planning_execution
```

## Usage

```sh
$ rtmlaunch nextage_ros_bridge nextage_ros_bridge_simulation.launch
$ roslaunch nextage_moveit_planning_execution planning_execution.launch
$ rosrun nextage_moveit_planning_execution moveit_command_sender.py
```

![](http://daikimaekawa.github.io/images/moveit/command_sender_plan3.jpg)

## License

Copyright (c) 2014, [Daiki Maekawa](http://daikimaekawa.strikingly.com/). (MIT License)

See LICENSE for more info.
