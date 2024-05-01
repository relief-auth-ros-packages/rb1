cmake_minimum_required(VERSION 2.8.3)
project(robot_local_control_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  procedures_msgs
  geometry_msgs
  std_msgs
  robotnik_msgs 
)

add_message_files(
   FILES
   LeaveShower.msg
   EnterShower.msg
   Uncharge.msg
   Place.msg
   Charge.msg
   Move.msg
   Pick.msg
   Dock.msg
   GoTo.msg
   StatusArray.msg
   Status.msg
   Pose2DStamped.msg
   Twist2D.msg
   LocalizationStatus.msg
   MissionStatus.msg
   NavigationStatus.msg
   RobotStatus.msg
   SensorStatus.msg
#   SetControlState.msg
   Missions.msg
# Mission command & params
  MissionCommand.msg
  MissionParamInt.msg
  MissionParamFloat.msg
  MissionParamBool.msg
  MissionParamString.msg
)

add_service_files(
  FILES
  LeaveShowerPetition.srv
  EnterShowerPetition.srv
  UnchargePetition.srv
  PlacePetition.srv
  ChargePetition.srv
  MovePetition.srv
  PickPetition.srv
  DockPetition.srv
  GoToPetition.srv
  StatusPetition.srv
  SetControlState.srv
  Cancel.srv
  SimpleGoToWithValidation.srv
  MissionCommandPetition.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
# )

generate_messages(
  DEPENDENCIES
  procedures_msgs
  geometry_msgs
  std_msgs
  robotnik_msgs 
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_local_control_msgs
 CATKIN_DEPENDS message_runtime std_msgs geometry_msgs procedures_msgs robotnik_msgs
#  DEPENDS system_lib
)

