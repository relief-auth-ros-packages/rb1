#ifndef CULTUREID_RFID_FOLLOWING_H
#define CULTUREID_RFID_FOLLOWING_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <limits>
#include <string>
#include <sstream>
#include <stdlib.h>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "actionlib_msgs/GoalID.h"
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Empty.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include "visualization_msgs/MarkerArray.h"
#include "range_libc/includes/RangeLib.h"
#include "cultureid_rfid_following/AoASrv.h"


class CRFIDFollow
{
  private:
    ros::NodeHandle nodehandle_;

    // The move base goal action client
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_goal_client_;

    // Controls the rate at which a new orientation is read from a file
    double callback_frequency_;
    ros::Timer periodic_callback_timer_;

    // List publishers / service clients
    ros::Publisher goal_pub_;
    ros::Publisher goal_abortion_pub_;
    ros::Publisher velocity_pub_;
    ros::Publisher request_lost_tag_detection_pub_;
    ros::Publisher feasible_goals_vis_pub_;
    ros::Publisher free_goal_pub_;

    ros::ServiceClient make_plan_client_;
    ros::ServiceClient aoa_client_;
    ros::ServiceClient clear_costmaps_client_;

    // List subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber lost_tag_detected_sub_;
    ros::Subscriber orientation_subscriber_;

    // List subscribed topics
    std::string pose_topic_;
    std::string velocity_topic_;
    std::string scan_topic_;
    std::string map_topic_;
    std::string request_lost_tag_topic_;
    std::string lost_tag_detected_topic_;

    // Feasible goals (allowable goals)
    std::vector< std::pair<double,double> > feasible_goals_;
    unsigned int current_feasible_goal_idx_;
    unsigned int goals_counter_;
    bool is_goal_active_;
    double robot_position_to_goal_distance_threshold_;

    // Type of robot running this node
    std::string robot_type_;
    double robot_radius_;

    // List logfile filenames
    std::string goal_orientation_filename_;
    std::string pose_robot_filename_;
    std::string pose_filename_;
    std::string velocity_filename_;
    std::string antennas_polarities_filename_;

    // Active readers
    unsigned int num_readers_;
    std::vector<bool> active_readers_;
    std::vector<std::string> readers_macs_;

    // The distance the free goal is put in relation to the robot
    double free_goal_distance_;

    // Active antennas per reader
    std::vector< std::vector<bool> > readers_active_antennas_;

    // Pose per antenna and per reader
    std::vector< std::vector< std::vector<double> > > reader_antenna_poses_;

    // Latest pose, most up-to-date (say timestep t)
    geometry_msgs::PoseWithCovarianceStamped latest_pose_;

    // The pose that corresponds to the most recent reliable aoa estimate
    geometry_msgs::PoseWithCovarianceStamped latest_reliable_pose_;

    // The pose at the previous orientation estimation step (say timestep t-1)
    geometry_msgs::PoseWithCovarianceStamped previous_pose_;

    // How many poses have been published
    unsigned int poses_counter_;

    // Latest orientation update and its timestamp
    double latest_timestamp_;
    double latest_orientation_;
    double latest_rssi_;
    double latest_reliable_orientation_;

    // Goal-to-goal reliability flags: At each call of the aoa service between
    // two goals, it stores a reliability of estimate flag for that call.
    // Resets after the generation of a new goal
    std::vector<bool> g2g_reliability_flags_;
    bool in_goal_radius_;

    // Previous orientation estimate
    std::vector<double> aoa_estimates_;
    std::vector<double> aoa_alternative_estimates_;
    std::vector<double> aoa_past_estimates_;

    // The map
    bool received_map_;
    nav_msgs::OccupancyGrid map_;

    // The map converted to a suitable structure for 3rd-party lib ray-casting
    ranges::OMap omap_;
    std::string map_png_file_;

    // The latest scan
    bool received_scan_;
    sensor_msgs::LaserScan::Ptr latest_scan_;

    // The orientation of the lidar
    std::string laser_z_orientation_;

    // 3rd-party ray-caster (range_libc)
    ranges::RayMarching rm_;

    // Autorotation flag
    bool do_autorotation_;


    // Transforms
    //
    // static, cached
    tf::Transform base_to_laser_;

    // static, cached, calculated from base_to_laser_
    tf::Transform laser_to_base_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::StampedTransform map_to_base_tf_;



    // Functions

    void abortGoal();

    unsigned int associateFreeGoalToFeasibleGoal(
      const std::pair<double,double>& free_goal,
      const std::vector< std::pair<double,double> >& feasible_goals);

    void autorotate();

    std::vector<geometry_msgs::PoseStamped> calculateAntennaPose(
      const geometry_msgs::PoseWithCovarianceStamped& robot_pose);

    bool callPlanningService(ros::ServiceClient serviceClient,
      nav_msgs::GetPlan& srv);

    cultureid_rfid_following::AoASrv constructAOARequest();

    std::pair<double,double> determineFreeGoal(
      const geometry_msgs::PoseWithCovarianceStamped& current_pose,
      const double& free_goal_distance,
      const double& aoa_dyaw);

    double pairSQDistance(const std::pair<double,double>& p1,
      const std::pair<double,double>& p2);

    double extractYawFromPose(
      const geometry_msgs::Pose& pose);
    double extractYawFromPose(
      const geometry_msgs::PoseWithCovarianceStamped& pose);

    void fillPathRequest(nav_msgs::GetPlan::Request* request,
      const double& start_x, const double& start_y,
      const double& goal_x, const double& goal_y);

    bool getBaseToLaserTf(const std::string& frame_id);

    geometry_msgs::Pose
      getCurrentLaserPose(const geometry_msgs::Pose& robot_pose);

    void initLogfiles();
    std::istream& ignoreline(std::ifstream& in, std::ifstream::pos_type& pos);

    bool isGoalFeasible(const std::pair<double,double>& goal,
      const bool& do_return_plan, nav_msgs::Path::Ptr plan_to_goal);

    // List callbacks
    void aoaOrientationCallback(const std_msgs::Float64& msg);
    void lostTagDetectedCallback(const std_msgs::Empty& msg);
    void mapCallback(const nav_msgs::OccupancyGrid& map_msg);
    void periodicCallback(const ros::TimerEvent& event);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void scanCallback(const sensor_msgs::LaserScan::Ptr& scan_msg);
    void velocityCallback(const geometry_msgs::Twist& msg);

    void publishAllFeasibleGoalsMarkers();
    void publishGoalWithOrientation(const std::pair<double,double>& goal,
      const double& orientation,
      const ros::Publisher& goal_pub);

    unsigned int numRaysFromAngleRange(const double& angle_min,
      const double& angle_max, const int& num_rays, const double& new_range);

    // Read the last entry from the goal orientations file --- level 1 function
    void readLastLineFromGoalOrientationsFile(
      const char* filepath,
      double* timestamp,
      double* orientation);

    // Read the last entry from the goal orientations file --- level 0 function
    std::string readLastLineFromGoalOrientationsFile(std::ifstream& in);

    // Init / helpers
    void loadParams();
    void loadReaderAndAntennaParams();
    void loadFeasibleGoalsList();

    void requestLostTagDetection();

    sensor_msgs::LaserScan::Ptr scanMap(
      const geometry_msgs::PoseWithCovarianceStamped::Ptr& robot_pose);
    void sendGoal(const double& x, const double& y, const double& yaw);
    void sendGoalSimple(const double& x, const double& y, const double& yaw);
    int sgn(const double& x);

    std::vector<double>
      unwrapAntennaPose(const geometry_msgs::PoseStamped& antenna_pose);

    void wrapAngle(double& angle);
    void writeReadersAntennasPolarities();


  public:
    CRFIDFollow(void);
    ~CRFIDFollow(void);
};

#endif // CULTUREID_RFID_FOLLOWING_H
