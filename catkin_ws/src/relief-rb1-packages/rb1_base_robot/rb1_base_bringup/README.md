# rb1_base_robot
includes all the packages required for the robot control.

## rb1_base_bringup
This package contains the launch files needed to run the RB1 robot.

We have to create a file named **robot_params.env**. Besides, we have to include `source ~/robot_params.env` to the *.bashrc* file. This file contains environment variables that establish the actual components of the robot. For example: 
```
export ROBOT_ID=rb1_base
# rb1_base.urdf.xacro
export ROBOT_XACRO=versions/rb1_base.urdf.xacro
# true, false
export ROBOT_HAS_LASER=true
# sick_tim561, hokuyo_ug01, hokuyo_ust
export ROBOT_LASER_MODEL=hokuyo_ust
#export ROBOT_LASER_PORT=/dev/ttyACM0
#export ROBOT_LASER_IP=192.168.0.10
# true, false
export ROBOT_HAS_RGBD_CAMERA=true
export ROBOT_RGBD_MODEL=orbbec
# true, false
# 24V motors: 12.52, 48V motors: 9.56, 24V-RB1 modified: 7.5
export ROBOT_GEARBOX=12.52
# ps3, ps4, logitechf710
export ROBOT_PAD_MODEL=ps4
```
##### Explanation of each environment variable
- `ROBOT_ID` indicates the name of the robot. This is the name of the namespace under all the nodes will be working. This is also used as the prefix of all the subcomponents.(*summit_xl*)
- `ROBOT_XACRO` indicates the path where the xacro file is. (inside the robot folder in robot_description)(*rb1_base.urdf.xacro*)
- `ROBOT_LASER_MODEL` indicates the model of the laser that the robot is using. The model is the name of the launch file.(*sick_tim561/hokuyo_ug01/hokuyo_ust*)
- `ROBOT_HAS_LASER` indicates if the robot has a laser in front. (*true/false*)
- `ROBOT_HAS_RGBD_CAMERA` indicates if the robot has a front rgbd camera. (*true/false*)
- `ROBOT_GEARBOX` establishes the motor gearbox value. (*24V: 12.52 | 48V: 9.56* | 24V-SPECIAL: 7.5*)
- `ROBOT_PAD_MODEL` indicates the model of the pad. (*ps3 | ps4 | logitechf710*)

