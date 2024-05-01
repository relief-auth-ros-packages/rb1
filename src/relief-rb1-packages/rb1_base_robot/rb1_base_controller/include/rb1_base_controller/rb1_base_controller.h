#include <boost/circular_buffer.hpp>

#include <controller_interface/controller.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>
#include <sensor_msgs/Imu.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <robotnik_msgs/set_mode.h>
#include <robotnik_msgs/get_mode.h>
#include <robotnik_msgs/set_odometry.h>

#include <robotnik_msgs/set_digital_output.h>
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_msgs/SetElevator.h>
#include <robotnik_msgs/ElevatorStatus.h>
#include <robotnik_msgs/ElevatorAction.h>
#include <robotnik_msgs/enable_disable.h>

#include <actionlib/server/simple_action_server.h>
#include <robotnik_msgs/SetElevatorAction.h>
#include <string>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

using namespace std;

namespace rb1_base_controller
{
enum
{
  RIGHT_TRACTION_JOINT = 0,
  LEFT_TRACTION_JOINT = 1,
  BEGIN_TRACTION_JOINT = 0,
  END_TRACTION_JOINT = 2,
  NUMBER_OF_JOINTS = 2,
  NUMBER_OF_WHEELS = 2,
};

struct KinematicLimits
{
  double linear_speed;
  double linear_acceleration;
  double linear_deceleration;
  double angular_speed;
  double angular_acceleration;
  double angular_deceleration;
};

class RB1BaseController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>

/* The standard way on indigo is to inherit from controller_interface::Controller<T>,
where T is a JointInterface. So a controller can access to only one type of JointInterface

As a hack, if we inherit from ControllerBase instead, we can access to two different JointInterfaces
In that case, we need to implement:
    initRequest, which receives a RobotHW instead of a hardware_interface::XJointInterface, from where the interfaces
can be accessed
    getHardwareInterfaceType, which returns a string with the main type of JointInterface. In our case, it is a
VelocityJointInterface
*/

{
public:
  RB1BaseController();

  /**
  */

  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  /**
   * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
   * \param time   Current time
   * \param period Time since the last called to update
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Starts controller
   * \param time Current time
   */
  void starting(const ros::Time& time);

  /**
   * \brief Stops controller
   * \param time Current time
   */
  void stopping(const ros::Time& /*time*/);

  virtual std::string getHardwareInterfaceType() const;

private:
  // Services

  // Control stuff
  std::vector<hardware_interface::JointHandle> joints_;  // joint handles: to read current state and send commands
  std::vector<std::string> joint_names_;                 // joint names: to get the handle of each joint

  std::vector<std::pair<double, double> > joint_limits_;  // lower, upper limits

  std::vector<double> joint_states_;       // current joint state: position or velocity
  std::vector<double> joint_states_mean_;  // current joint state mean: used to calculate the reference according to the
                                           // constraints
  std::vector<double> joint_references_;   // current reference for each joint
  std::vector<double> joint_commands_;  // current command to be sent: may differ from reference is the wheels are not
                                        // in position or if the watchdog times out

  std::vector<boost::circular_buffer<double> > joint_states_history_;  // used to calculate the current joint state as
                                                                       // the mean of the previous joint states
  unsigned int joint_states_history_size_;                             // size of the joint history

  // Data
  geometry_msgs::Twist received_cmd_;  // holds last velocity command
  geometry_msgs::Twist current_cmd_;  // hold current used command (limited). it is updated on the limitCommand function
                                      // and in the writeJointCommands, because if the direction wheels are not in
                                      // position, the traction reference is 0
  nav_msgs::Odometry odom_;           // holds odometry
  geometry_msgs::Pose2D robot_pose_;  // holds robot position
  ros::Time cmd_last_stamp_;          // holds last velocity command time stamp, used to check the watchdog
  bool cmd_watchdog_timedout_;        // true is the watchdog has been activated
  robotnik_msgs::inputs_outputs received_io_;  // holds last io msg
  ros::Time io_last_stamp_;                    // holds the last io msg stamp

  ros::Duration cmd_watchdog_duration_;  // time that has to pass to activate the watchdog

  ros::Time imu_last_stamp_;             // holds last velocity command time stamp, used to check the watchdog
  bool imu_watchdog_timedout_;           // true is the watchdog has been activated
  ros::Duration imu_watchdog_duration_;  // time that has to pass to activate the watchdog
  double in_motion_timer_;

  bool odom_broadcast_tf_;             // if true, odom must be published also in tf
  ros::Duration odom_publish_period_;  // time between odometry publication updates
  ros::Time odom_last_sent_;           // to check if the odometry must be sent
  ros::Time odom_last_update_;         // to use in the odometry calculation

  // Wheels configuration
  double track_width_;     // distance between right and left wheels
  double wheel_diameter_;  // wheel diamater, to convert from angular speed to linear

  // Speed and acceleration limits
  KinematicLimits limits_elevator_down_;
  KinematicLimits limits_elevator_up_;
  KinematicLimits limits_hard_brake_;
  bool hard_brake_;

  bool motion_odometry_;

  std::string imu_topic_;  // name of the topic which is publishing sensor_msgs/Imu messages
  bool bFirstYawRead_;
  bool bYawSensor_;
  double init_yaw_;    // Init yaw from the yaw sensor (gyro or imu)
  double last_yaw_;    // Last yaw read from the yaw sensor (gyro or imu)
  double delta_yaw_;   // Yaw increment in one period
  double motion_yaw_;  // Variable that integrates yaw only when the robot is in motion
  double imu_yaw_;

  // ROS stuff
  std::string controller_name_;   // node name,
  std::string command_topic_;     // topic from where velocity commands are read
  std::string odom_topic_;        // name of the topic where the odometry is published
  std::string odom_frame_;        // name of the frame associated to the odometry
  std::string robot_base_frame_;  // name of the frame associated to the robot. odom_frame_ is published as it's parent
  bool reverse_logic_;

  double odom_linear_velocity_covariance_;
  double odom_angular_velocity_covariance_;
  double odom_position_covariance_;
  double odom_orientation_covariance_;

  std::string set_digital_output_service_hw_;  // name of the topic where the odometry is published
  // Publishers
  ros::Publisher odom_pub_;                    // topic publisher where the odometry is published
  ros::Publisher elevator_status_pub_;         // topic publisher for the elevator
  ros::Publisher joint_states_pub_;            // topic publisher for the elevator joint state
  ros::Publisher enabled_pub_;            		// topic publisher to publish if the controller is enabled
  ros::Publisher elevator_pub_;
  // Subscribers
  ros::Subscriber cmd_vel_sub_;  // topic subscriber to receive velocity commands
  ros::Subscriber imu_sub_;      // IMU subscriber
  ros::Subscriber io_sub_;       // I/O subscriber

  tf::TransformBroadcaster* transform_broadcaster_;  // to publish odom frame
  // Service Clients
  ros::ServiceClient set_digital_output_client;
  // Service Servers
  ros::ServiceServer set_odometry_srv_;
  ros::ServiceServer set_elevator_srv_;
  ros::ServiceServer enable_srv_;

  bool setOdometrySrvCallback(robotnik_msgs::set_odometry::Request& request,
                              robotnik_msgs::set_odometry::Response& response);
  bool setElevatorSrvCallback(robotnik_msgs::SetElevator::Request& req, robotnik_msgs::SetElevator::Response& res);
  bool enableSrvCallback(robotnik_msgs::enable_disable::Request& req, robotnik_msgs::enable_disable::Response& res);

  // Elevator
  // Strnig param elevator joint
  std::string elevator_joint_name_;
  // Boolean param to know if the robot has elevator
  bool has_elevator_;
  ros::Time last_elevator_action_time;  // Time of the last elevator action
  
  robotnik_msgs::ElevatorAction elevator_action_, elevator_ongoing_action_;  // saves the action to carry out
  robotnik_msgs::ElevatorStatus elevator_status_;      // saves the elevator status for publishing purposes
  ros::Time elevator_loop_last_execution_;             // to check if the elevator status must be sent
  ros::Time elevator_init_action_time_;                // Time when the action
  ros::Duration elevator_status_control_loop_period_;  // control loop period
  ros::Duration elevator_action_command_timeout_;      // timeout processing an action received
  int elevator_digital_output_up_;
  int elevator_digital_output_down_;
  int elevator_digital_input_up_;
  int elevator_digital_input_down_;
  ros::Time elevator_action_init_time;  // Time when the action starts
  double elevator_position_up_, elevator_position_down_;
  sensor_msgs::JointState joint_states_msg;
  double elevator_current_position_;  // The position to publish
  bool enabled_; 						//Flag to accept any type of command
  
  //
  void readKinematicLimits(KinematicLimits& limit, ros::NodeHandle& controller_nh, const std::string& base = "");
  void readJointStates();
  void writeJointCommands();
  void limitCommand(double period);
  void updateJointStateHistoryMean();
  void updateJointReferences();
  void setJointVelocityReferenceBetweenJointLimits(std::vector<double>& wheel_speed);
  void updateOdometry();
  bool inMotion();
  void publishOdometry();
  void publishElevatorStatus();
  void elevatorControlLoop(const ros::Time& time);
  void switchToElevatorState(string new_state);

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  void ioCallback(const robotnik_msgs::inputs_outputsConstPtr& msg);
  int setDigitalOutput(int number, bool value);

  actionlib::SimpleActionServer<robotnik_msgs::SetElevatorAction>* elevator_action_server_;
  robotnik_msgs::SetElevatorFeedback elevator_action_feedback_;
  robotnik_msgs::SetElevatorResult elevator_action_result_;

  void executeElevatorCallback(const robotnik_msgs::SetElevatorGoalConstPtr& goal);
};
PLUGINLIB_EXPORT_CLASS(rb1_base_controller::RB1BaseController, controller_interface::ControllerBase);
}
