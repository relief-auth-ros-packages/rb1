#include "marker_replacement_node/marker_replacement.h"

/*******************************************************************************
 * Constructor. Initializes publishers/subscribers.
 */
MarkerReplacement::MarkerReplacement(void)
{
  // This publisher publishes the map's vertices.
  polygon_points_pub_ = nodehandle_.advertise<geometry_msgs::PointStamped>(
    "/clicked_point", 6);

  // Subscribe to the map_metadata topic.
  // We will extract the relevant
  // origin.position.*, resolution, width, height of the map from it
  map_metadata_sub_ = nodehandle_.subscribe("/map_metadata", 1,
    &MarkerReplacement::mapMetadataCallback, this);

  // Call this service in order to begin exploration without having to provide
  // the exploration polygon by hand through RViz
  init_exploration_srv_ = nodehandle_.advertiseService(ros::this_node::getName()
    + "/init_exploration_srv", &MarkerReplacement::initExploration, this);

  // Service called internally for when the map is resized and the explorer's
  // bounds need to be updated
  update_exploration_client_ = nodehandle_.serviceClient<std_srvs::Trigger>(
    ros::this_node::getName() + "/init_exploration_srv");

  is_frontier_exploration_on_ = false;

  ROS_INFO("[MarkerReplacement] Node initialised");
}



/*******************************************************************************
 * Destructor
 */
MarkerReplacement::~MarkerReplacement(void)
{
  ROS_INFO("[MarkerReplacement] Node destroyed");
}



/*******************************************************************************
 * Construction of the rectangle that outlines the map
 * Note that it may be the case that the map's height and width cover more
 * than the actual traversable space for the robot and, therefore, point D
 * may actually end up falling into non-traversable space. (Is it a problem?)
 * A---------B
 * |         |
 * |    D    |
 * |         |
 * O---------C
 */
void MarkerReplacement::constructPolygon(void)
{
  // A small quantity to add or subtract from the vertices so that all of them
  // are within the map; otherwise frontier_explorer crashes.
  // For details see:
  // https://github.com/paulbovbel/frontier_exploration/issues/37
  double e = 0.01;

  // Commence construction of boundary vertices
  geometry_msgs::PointStamped O;
  O.header.seq = 0;
  O.header.stamp = ros::Time::now();
  O.header.frame_id = "map";
  O.point.x = map_metadata_struct_.origin_position_x_ + e;
  O.point.y = map_metadata_struct_.origin_position_y_ + e;
  polygon_points_pub_.publish(O);

  geometry_msgs::PointStamped A;
  A.header.seq = 1;
  A.header.stamp = ros::Time::now();
  A.header.frame_id = "map";
  A.point.x = map_metadata_struct_.origin_position_x_ + e;
  A.point.y = map_metadata_struct_.origin_position_y_ +
    map_metadata_struct_.resolution_ * map_metadata_struct_.height_ - e;
  polygon_points_pub_.publish(A);

  geometry_msgs::PointStamped B;
  B.header.seq = 2;
  B.header.stamp = ros::Time::now();
  B.header.frame_id = "map";
  B.point.x = map_metadata_struct_.origin_position_x_ +
    map_metadata_struct_.resolution_ * map_metadata_struct_.width_ - e;
  B.point.y = map_metadata_struct_.origin_position_y_ +
    map_metadata_struct_.resolution_ * map_metadata_struct_.height_ - e;
  polygon_points_pub_.publish(B);

  geometry_msgs::PointStamped C;
  C.header.seq = 3;
  C.header.stamp = ros::Time::now();
  C.header.frame_id = "map";
  C.point.x = map_metadata_struct_.origin_position_x_ +
    map_metadata_struct_.resolution_ * map_metadata_struct_.width_  - e;
  C.point.y = map_metadata_struct_.origin_position_y_ + e;
  polygon_points_pub_.publish(C);

  geometry_msgs::PointStamped OO;
  OO.header.seq = 4;
  OO.header.stamp = ros::Time::now();
  OO.header.frame_id = "map";
  OO.point.x = O.point.x;
  OO.point.y = O.point.y;
  polygon_points_pub_.publish(OO);

  geometry_msgs::PointStamped D;
  D.header.seq = 5;
  D.header.stamp = ros::Time::now();
  D.header.frame_id = "map";
  D.point.x = (O.point.x + C.point.x) / 2;
  D.point.y = (O.point.y + A.point.y) / 2;
  polygon_points_pub_.publish(D);
}



/*******************************************************************************
 * Stores the map's metadata to a struct for (later) usage by the polygon
 * construction service
 */
bool MarkerReplacement::initExploration(std_srvs::TriggerRequest& req,
  std_srvs::TriggerResponse& res)
{
  ROS_INFO("[MarkerReplacement] Service init_exploration called");

  is_frontier_exploration_on_ = true;

  constructPolygon();

  res.success = true;
  res.message = "Polygon coordinates created from map, and sent to trigger exploration";
  return true;
}



/*******************************************************************************
 * Stores the map's metadata to a struct for (later) usage by the polygon
 * construction service.
 * Updates the frontier explorer exploration polygon when maps are resized.
 */
void MarkerReplacement::mapMetadataCallback(const nav_msgs::MapMetaData& msg)
{
  unsigned int previous_map_height = map_metadata_struct_.height_;
  unsigned int previous_map_width = map_metadata_struct_.width_;

  map_metadata_struct_.origin_position_x_ = msg.origin.position.x;
  map_metadata_struct_.origin_position_y_ = msg.origin.position.y;
  map_metadata_struct_.origin_position_z_ = msg.origin.position.z;

  map_metadata_struct_.origin_orientation_x_ = msg.origin.orientation.x;
  map_metadata_struct_.origin_orientation_y_ = msg.origin.orientation.y;
  map_metadata_struct_.origin_orientation_z_ = msg.origin.orientation.z;
  map_metadata_struct_.origin_orientation_w_ = msg.origin.orientation.w;

  map_metadata_struct_.resolution_ = msg.resolution;
  map_metadata_struct_.height_ = msg.height;
  map_metadata_struct_.width_ = msg.width;

  // Upon receipt of the first map, the exploration shall be carried out by
  // calling the initExploration service (by hand only).
  // Upon receipt of the subsequent maps (where the size of the map
  // is augmented) the exploration boundary shall be updated automatically.
  if ( (previous_map_height > 0 || previous_map_width > 0) &&
    (previous_map_height != map_metadata_struct_.height_ ||
     previous_map_width != map_metadata_struct_.width_) &&
    is_frontier_exploration_on_ )
  {
    constructPolygon();
  }

}
