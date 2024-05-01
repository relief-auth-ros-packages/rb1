#ifndef MARKERREPLACEMENT_H
#define MARKERREPLACEMENT_H

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/MapMetaData.h"
#include "std_srvs/Trigger.h"

class MarkerReplacement
{
  private:
    ros::NodeHandle nodehandle_;
    ros::Publisher polygon_points_pub_;
    ros::Subscriber map_metadata_sub_;
    ros::ServiceServer init_exploration_srv_;
    ros::ServiceClient update_exploration_client_;

    bool is_frontier_exploration_on_;

    struct MapMetadataStruct
    {
      double origin_position_x_;
      double origin_position_y_;
      double origin_position_z_;
      double origin_orientation_x_;
      double origin_orientation_y_;
      double origin_orientation_z_;
      double origin_orientation_w_;
      double resolution_;
      unsigned int height_;
      unsigned int width_;

      MapMetadataStruct() : origin_position_x_(0.0),
        origin_position_y_(0.0),
        origin_position_z_(0.0),
        origin_orientation_x_(0.0),
        origin_orientation_y_(0.0),
        origin_orientation_z_(0.0),
        origin_orientation_w_(0.0),
        resolution_(0.0),
        height_(0),
        width_(0) {}
    } map_metadata_struct_;

    void constructPolygon(void);

    bool initExploration(std_srvs::TriggerRequest& req,
      std_srvs::TriggerResponse& res);

    void mapMetadataCallback(const nav_msgs::MapMetaData& msg);


  public:
    MarkerReplacement(void);
    ~MarkerReplacement(void);

};

#endif // MARKERREPLACEMENT_H
