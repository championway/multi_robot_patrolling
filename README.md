# Multi-Robot Patrolling

## Installation

### Dependencies

```
None
```

Note:

### ROS Package

```
$ git clone https://github.com/championway/multi_robot_patrolling
$ cd multi_robot_patrolling
$ source environment.sh
$ cd catkin_ws
$ catkin_make
```

## Run the code

### for Master (Can execute on desktop)
```
$ roslaunch master master_node.launch p_num:=[number_of_patrolling_node]
```

### for Robot (Execute on robots)

```
$ roslaunch demo patrolling.launch veh:=[your_robot_name]
```
