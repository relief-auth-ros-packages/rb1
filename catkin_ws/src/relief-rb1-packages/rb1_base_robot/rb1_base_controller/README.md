# rb1_base_controller

Differential controller plugin compatible with [ros_control](http://wiki.ros.org/ros_control) architecture.

This component is in charge of setting & reading the robot motor joints.

## 1- Bringup

This plugin is loaded from the [rb1_base_control](/rb1_base_control) package.

```
>roslaunch rb1_base_control rb1_base_control.launch
```

## 2- Parameters

## 3- Topics & Services

### 3.1 Subscribers

 * robotnik_base_control/cmd_vel (geometry_msgs/Twist)
 * robotnik_base_control/elevator_status (robotnik_msgs/ElevatorStatus)
 * robotnik_base_control/enabled (std_msgs/Bool)
 * robotnik_base_control/odom (navigation_msgs/Odometry)

### 3.2 Service Servers

 * robotnik_base_control/enable (robotnik_msgs/enable_disable)
 * robotnik_base_control/set_elevator (robotnik_msgs/SetElevator)
 * set_odometry (robotnik_msgs/set_odometry)

### 3.3 Action servers

 * robotnik_base_control/set_elevator (robotnik_msgs/SetElevator)
   * robotnik_base_control/set_elevator/cancel
   * robotnik_base_control/set_elevator/feedback
   * robotnik_base_control/set_elevator/goal
   * robotnik_base_control/set_elevator/result
   * robotnik_base_control/set_elevator/status

## 4- Examples

### Setting the elevator position

The elevator position can be set using the service [set_elevator](https://github.com/RobotnikAutomation/robotnik_msgs/blob/master/srv/SetElevator.srv). Usage: 
```
rosservice call /rb1_base/rb1_base_control/set_elevator robotnik_msgs/SetElevator "action: action: 0"
```
To raise, lower o stop the elevator set the action value to 1 / -1 / 0  respectively
