/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */
/* Author: Dinesh Thakur - Modified for waypoint mapping */

#include <stdlib.h>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <ctime>
#include <OGRE/OgreSceneManager.h>
#include <rviz/display_context.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "waypoint_mapping_tool.h"

#include <tf/tf.h>

#include <QFileDialog>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace waypoint_mapping_plugin
{

WaypointFrame::WaypointFrame(rviz::DisplayContext *context, std::map<int, Ogre::SceneNode* >* map_ptr, interactive_markers::InteractiveMarkerServer* server, QWidget *parent, WaypointMappingTool* wp_tool)
  : QWidget(parent)
  , context_(context)
  , ui_(new Ui::WaypointMappingWidget())
  , sn_map_ptr_(map_ptr)
  , server_(server)
  , mapping_resolution_(0.01)
  , wp_mapping_tool_(wp_tool)
{
  scene_manager_ = context_->getSceneManager();

  // set up the GUI
  ui_->setupUi(this);

  //connect the Qt signals and slots
  connect(ui_->start_mapping_button, SIGNAL(clicked()), this, SLOT(startMappingButtonClicked()));
  connect(ui_->store_map_button, SIGNAL(clicked()), this, SLOT(storeMapButtonClicked()));
  connect(ui_->resolution_slider, SIGNAL(valueChanged(int)), this, SLOT(resolutionChangedSlider(int)));

  map_sub_ = nh_.subscribe("/map", 1, &WaypointFrame::mapCallback, this);
}

WaypointFrame::~WaypointFrame()
{
  delete ui_;
  sn_map_ptr_ = NULL;
}

void WaypointFrame::enable()
{
  show();
}

void WaypointFrame::disable()
{
  hide();
}

void WaypointFrame::resolutionChangedSlider(int i)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  mapping_resolution_ = static_cast<double>(i)/100;

  setResolutionLabel();
}

void WaypointFrame::setResolutionLabel()
{
  std::ostringstream stringStream;
  stringStream << static_cast<int>(mapping_resolution_ * 100);
  std::string st = stringStream.str();

  ui_->resolution_value_label->blockSignals(true);
  ui_->resolution_value_label->setText(QString::fromStdString(st));
  ui_->resolution_value_label->blockSignals(false);
}

void WaypointFrame::startMappingButtonClicked()
{
  std::ostringstream stringStream;
  stringStream << mapping_resolution_;
  std::string mapping_resolution_str = stringStream.str();

  std::string exec_str1 =
    "roslaunch relief_devel avanti_live.launch task:=mapping slam_resolution:=" +
    mapping_resolution_str + " &";
  ROS_INFO("Issuing $ %s\n", exec_str1.c_str());

  system(exec_str1.c_str());
}

void WaypointFrame::storeMapButtonClicked()
{
  // Get map_suffix ------------------------------------------------------------
  std::string timestamp_now;
  getTimestampNow(timestamp_now);

  // Get map resolution --------------------------------------------------------
  std::ostringstream stringStream;
  stringStream << mapping_resolution_;
  std::string mapping_resolution_str = stringStream.str();

  // Get map origin ------------------------------------------------------------
  std::ostringstream stringStream_x;
  stringStream_x << -map_origin_.position.x;
  std::ostringstream stringStream_y;
  stringStream_y << -map_origin_.position.y;

  std::pair<std::string, std::string> map_origin_str;
  map_origin_str.first = stringStream_x.str();
  map_origin_str.second = stringStream_y.str();

  // Generate map name ---------------------------------------------------------
  std::string map_name =
    "map_" + timestamp_now + "_" + mapping_resolution_str;

  // Store `map_name`.pgm and `map_name`.yaml ----------------------------------
  std::string exec_str1 =
    "cd /home/rb1/catkin_ws/src/relief-devel/maps; rosrun map_server map_saver -f " + map_name;
  ROS_INFO("Issuing $ %s\n", exec_str1.c_str());

  system(exec_str1.c_str());

  // Generate the avanti_live launcher -----------------------------------------
  generateLauncher2(map_name, map_origin_str, mapping_resolution_str);

  // Make the origin be [0,0,0] in `map_name`.yaml -----------------------------
  modifyYAMLOrigin(map_name);

  ROS_INFO("Done generating the map and its relevant files");
}


/* Added by li9i 02/11/2021
*/
void WaypointFrame::getTimestampNow(std::string& ts_str)
{
  time_t curr_time;
  tm * curr_tm;
  char date_c[100];
  char time_c[100];

  time(&curr_time);
  curr_tm = localtime(&curr_time);

  strftime(date_c, 50, "%Y_%m_%d", curr_tm);
  strftime(time_c, 50, "%T", curr_tm);

  std::string date_str = std::string(date_c);
  std::string time_str = std::string(time_c);
  ts_str = date_str + "_" + time_str;
}


void WaypointFrame::mapCallback(const nav_msgs::OccupancyGrid::Ptr& map)
{
  nav_msgs::MapMetaData info = map->info;
  geometry_msgs::Pose origin = info.origin;

  map_origin_ = origin;

  //printf("Origin: (%f,%f):\n", origin.position.x, origin.position.y);
}


/*
 * This is going to be ugly
 */
void WaypointFrame::generateLauncher(const std::string& map_name,
  const std::pair<std::string, std::string>& origin,
  const std::string& map_resolution)
{
  // The first segment
  std::vector<std::string> string_segment_first;

  string_segment_first.push_back(
    "<launch>");
  string_segment_first.push_back(
    "");
  string_segment_first.push_back(
    "  <!-- Init args -->");
  string_segment_first.push_back(
    "");
  string_segment_first.push_back(
    "  <!-- Available values for robot_type: {turtlebot|rb1}-->");
  string_segment_first.push_back(
    "  <arg name=\"robot_type\"              value=\"$(env ROBOT_TYPE)\" />");
  string_segment_first.push_back(
    "");
  string_segment_first.push_back(
    "  <!-- Available values for do_robot_bringup: {true|false} -->");
  string_segment_first.push_back(
    "  <arg name=\"do_robot_bringup\"        default=\"false\"/>");
  string_segment_first.push_back(
    "");
  string_segment_first.push_back(
    "  <!-- Available values for task: {localisation|mapping} -->");
  string_segment_first.push_back(
    "  <arg name=\"task\"                    default=\"localisation\"/>");
  string_segment_first.push_back(
    "");
  string_segment_first.push_back(
    "  <!-- Available values for slam_alg: {karto|gmapping|rtabmap} -->");
  string_segment_first.push_back(
    "  <arg name=\"slam_alg\"                default=\"karto\"     if=\"$(eval task == 'mapping')\"/>");
  string_segment_first.push_back(
    "");
  string_segment_first.push_back(
    "  <!-- Number of cameras used for 3D SLAM -->");
  string_segment_first.push_back(
    "  <arg name=\"num_cameras\"             default=\"$(env NUM_CAMERAS)\" />");
  string_segment_first.push_back(
    "");
  string_segment_first.push_back(
    "  <!-- Available values for slam_alg: {karto|gmapping} -->");
  string_segment_first.push_back(
    "  <arg name=\"slam_resolution\"         default=\"0.01\"      if=\"$(eval arg('task') == 'mapping' and (arg('slam_alg') == 'karto' or arg('slam_alg') == 'gmapping'))\"/>");
  string_segment_first.push_back(
    "  <arg name=\"slam_resolution\"         default=\"0.01\"      if=\"$(eval arg('task') == 'mapping' and arg('slam_alg') == 'rtabmap')\"/>");
  string_segment_first.push_back(
    "");
  string_segment_first.push_back(
    "  <!-- Available values for gp (determines the global planner used): {navfn|globalplanner|sbpl} -->");
  string_segment_first.push_back(
    "  <arg name=\"gp\"                      default=\"globalplanner\"/>");
  string_segment_first.push_back(
    "");
  string_segment_first.push_back(
    "  <!-- Available values for lp (determines the local planner used): {dwa|eband|teb} -->");
  string_segment_first.push_back(
    "  <arg name=\"lp\"                      default=\"teb\"/>");
  string_segment_first.push_back(
    "");
  string_segment_first.push_back(
    "  <!-- Avalailable values for use_explorer: {true|false} -->");
  string_segment_first.push_back(
    "  <arg name=\"use_explorer\"            default=\"false\" />");
  string_segment_first.push_back(
    "");
  string_segment_first.push_back(
    "  <!-- Avalailable values for use_viz {true|false} -->");
  string_segment_first.push_back(
    "  <arg name=\"do_viz\"                  default=\"false\" />");
  string_segment_first.push_back(
    "");
  string_segment_first.push_back(
    "  <!-- If performing localisation (and therefore the map is known apriori),");
  string_segment_first.push_back(
    "       set the map file name and its (given) resolution -->");
  string_segment_first.push_back(
    "");

  // The last segment
  std::vector<std::string> string_segment_last;
  string_segment_last.push_back(
    "  <!-- ********************************************************************* -->");
  string_segment_last.push_back(
    "  <!-- Launch robot -->");
  string_segment_last.push_back(
    "  <include file=\"$(find relief_devel)/launch/robot/$(arg robot_type)_bringup_live.launch\" if=\"$(arg do_robot_bringup)\"/>");
  string_segment_last.push_back(
    "");
  string_segment_last.push_back(
    "  <!-- ********************************************************************* -->");
  string_segment_last.push_back(
    "  <!-- Launch either localisation or mapping -->");
  string_segment_last.push_back(
    "  <include file=\"$(find relief_devel)/launch/$(arg task)/$(arg task).launch\">");
  string_segment_last.push_back(
    "    <arg name=\"robot_type\"            value=\"$(arg robot_type)\"/>");
  string_segment_last.push_back(
    "    <arg name=\"resolution\"            value=\"$(arg resolution)\" />");
  string_segment_last.push_back(
    "");
  string_segment_last.push_back(
    "    <arg name=\"slam_alg\"              value=\"$(arg slam_alg)\"                   if=\"$(eval task == 'mapping')\" />");
  string_segment_last.push_back(
    "    <arg name=\"num_cameras\"           value=\"$(arg num_cameras)\"                if=\"$(eval task == 'mapping' and arg('slam_alg') == 'rtabmap')\" />");
  string_segment_last.push_back(
    "");
  string_segment_last.push_back(
    "    <arg name=\"map_file\"              value=\"$(arg map_file)\"                   if=\"$(eval task == 'localisation')\" />");
  string_segment_last.push_back(
    "");
  string_segment_last.push_back(
    "    <arg name=\"initial_pose_x\"        value=\"$(arg estimate_initial_pose_x)\"    if=\"$(eval task == 'localisation')\" />");
  string_segment_last.push_back(
    "    <arg name=\"initial_pose_y\"        value=\"$(arg estimate_initial_pose_y)\"    if=\"$(eval task == 'localisation')\" />");
  string_segment_last.push_back(
    "    <arg name=\"initial_pose_yaw\"      value=\"$(arg estimate_initial_pose_yaw)\"  if=\"$(eval task == 'localisation')\" />");
  string_segment_last.push_back(
    "  </include>");
  string_segment_last.push_back(
    "");
  string_segment_last.push_back(
    "  <!-- ********************************************************************* -->");
  string_segment_last.push_back(
    "  <!-- Load navigation (move_base, global and local planner)-->");
  string_segment_last.push_back(
    "  <include file=\"$(find relief_devel)/launch/navigation/navigation.launch\" >");
  string_segment_last.push_back(
    "    <arg name=\"robot_type\"            value=\"$(arg robot_type)\"/>");
  string_segment_last.push_back(
    "    <arg name=\"num_cameras\"           value=\"$(arg num_cameras)\" />");
  string_segment_last.push_back(
    "    <arg name=\"gp\"                    value=\"$(arg gp)\"/>");
  string_segment_last.push_back(
    "    <arg name=\"lp\"                    value=\"$(arg lp)\"/>");
  string_segment_last.push_back(
    "    <arg name=\"task\"                  value=\"$(arg task)\"/>");
  string_segment_last.push_back(
    "    <arg name=\"resolution\"            value=\"$(arg resolution)\" />");
  string_segment_last.push_back(
    "  </include>");
  string_segment_last.push_back(
    "");
  string_segment_last.push_back(
    "  <!-- ********************************************************************* -->");
  string_segment_last.push_back(
    "  <!-- Load frontier explorer -->");
  string_segment_last.push_back(
    "  <group if=\"$(arg use_explorer)\">");
  string_segment_last.push_back(
    "    <include file=\"$(find relief_devel)/launch/exploration/frontier_exploration.launch\">");
  string_segment_last.push_back(
    "      <arg name=\"task\"                  value=\"$(arg task)\"/>");
  string_segment_last.push_back(
    "    </include>");
  string_segment_last.push_back(
    "");
  string_segment_last.push_back(
    "    <!-- Run rviz marker replacement for triggering rviz-less exploration -->");
  string_segment_last.push_back(
    "    <node pkg=\"relief_devel\" name=\"marker_replacement_node\" type=\"marker_replacement_node\" output=\"screen\" />");
  string_segment_last.push_back(
    "  </group>");
  string_segment_last.push_back(
    "");
  string_segment_last.push_back(
    "  <!-- ********************************************************************* -->");
  string_segment_last.push_back(
    "  <!-- Load visualisation -->");
  string_segment_last.push_back(
    "  <node name=\"rviz\" pkg=\"rviz\" type=\"rviz\" output=\"log\" args=\"-d $(find relief_devel)/configuration_files/rviz/relief_devel_$(arg gp)_$(arg lp).rviz\" if=\"$(arg do_viz)\" />");
  string_segment_last.push_back(
    "");
  string_segment_last.push_back(
    "</launch>");

  // Remove the resolution from map_name
  std::string::size_type start_pos = 0;
  while (map_name.find("_", start_pos) != std::string::npos)
    ++start_pos;

  std::string map_name_wr = map_name.substr(0, start_pos-1);
  printf("start pos = %zu\n", start_pos);
  printf("str = %s\n", map_name_wr.c_str());


  std::vector<std::string> middle_segment;
  middle_segment.push_back(
    "  <arg name=\"map_file\"                default=\"" + map_name_wr + "\"/>");
  middle_segment.push_back(
    "  <arg name=\"map_resolution\"          default=\"" + map_resolution +"\"      if=\"$(eval map_file == '" + map_name_wr + "')\" />");
  middle_segment.push_back(
    "");
  middle_segment.push_back(
    "  <!-- Abstract the map's resolution for independence from task -->");
  middle_segment.push_back(
    "  <arg name=\"resolution\"                value=\"$(arg map_resolution)\"  if=\"$(eval task == 'localisation')\" />");
  middle_segment.push_back(
    "  <arg name=\"resolution\"                value=\"$(arg slam_resolution)\" if=\"$(eval task == 'mapping')\" />");
  middle_segment.push_back(
    "  <arg name=\"estimate_initial_pose_x\"   value=\"" + origin.first + "\"  if=\"$(eval map_file == '" + map_name_wr + "')\"/>");
  middle_segment.push_back(
    "  <arg name=\"estimate_initial_pose_y\"   value=\"" + origin.second + "\"  if=\"$(eval map_file == '" + map_name_wr + "')\"/>");
  middle_segment.push_back(
    "  <arg name=\"estimate_initial_pose_yaw\" value=\"0.0\"       if=\"$(eval map_file == '" + map_name_wr + "')\"/>");


  // Store launcher
  std::string store_path = "/home/rb1/catkin_ws/src/relief-devel/launch/avanti_live_generated.launch";
  std::ofstream fil(store_path.c_str(), std::ios::trunc);

  if (fil.is_open())
  {
    for (unsigned int i = 0; i < string_segment_first.size(); i++)
      fil << string_segment_first[i] << std::endl;

    for (unsigned int i = 0; i < middle_segment.size(); i++)
      fil << middle_segment[i] << std::endl;

    for (unsigned int i = 0; i < string_segment_last.size(); i++)
      fil << string_segment_last[i] << std::endl;

    ROS_INFO("Done generating launcher file");
  }
  else
    ROS_ERROR("Could not store launcher");
}

/*
 * This is going to be pretty and more extensible.
 * The reasoning is the following:
 * There is one manually-crafted master launcher file (called avanti_live.launch)
 * atm (11/11/2021). This file is comprised of 3 segments: 1. from start to
 * one directive called `autogeneration_checkpoint_1` WHICH SHOULD BE MANUALLY
 * FILLED IN; 2. from this checkpoint to `autogeneration_checkpoint_2` (DITTO);
 * from this checkpoint to the end of the file.
 * The middle segment (autogeneration_checkpoint_1,autogeneration_checkpoint_2)
 * is the one that must be generated by this method and this package and
 * reflects the map's name, origin, and resolution.
 * What we do here is copy the first and third segment using the two directives
 * and generate the middle segment. We do not overwrite the original launcher
 * file.
 */
void WaypointFrame::generateLauncher2(const std::string& map_name,
  const std::pair<std::string, std::string>& origin,
  const std::string& map_resolution)
{
  // Remove resolution from map_name
  std::string::size_type start_pos = 0;
  while (map_name.find("_", start_pos) != std::string::npos)
    ++start_pos;

  std::string map_name_wr = map_name.substr(0, start_pos-1);

  // Generate the segment that is needed
  std::vector<std::string> generated_segment;
  generated_segment.push_back(
    "  <arg name=\"map_file\"                default=\"" + map_name_wr + "\"/>");
  generated_segment.push_back(
    "  <arg name=\"map_resolution\"          default=\"" + map_resolution +"\"      if=\"$(eval map_file == '" + map_name_wr + "')\" />");
  generated_segment.push_back(
    "");
  generated_segment.push_back(
    "  <!-- Abstract the map's resolution for independence from task -->");
  generated_segment.push_back(
    "  <arg name=\"resolution\"                value=\"$(arg map_resolution)\"  if=\"$(eval task == 'localisation')\" />");
  generated_segment.push_back(
    "  <arg name=\"resolution\"                value=\"$(arg slam_resolution)\" if=\"$(eval task == 'mapping')\" />");
  generated_segment.push_back(
    "  <arg name=\"estimate_initial_pose_x\"   value=\"" + origin.first + "\"  if=\"$(eval map_file == '" + map_name_wr + "')\"/>");
  generated_segment.push_back(
    "  <arg name=\"estimate_initial_pose_y\"   value=\"" + origin.second + "\"  if=\"$(eval map_file == '" + map_name_wr + "')\"/>");
  generated_segment.push_back(
    "  <arg name=\"estimate_initial_pose_yaw\" value=\"0.0\"       if=\"$(eval map_file == '" + map_name_wr + "')\"/>");
  generated_segment.push_back(
    "");



  // Non-generated launcher (the one we craft by hand)
  // Make sure not to overwrite it (should be ro)
  std::string ro_launcher_path =
    "/home/rb1/catkin_ws/src/relief-devel/launch/avanti_live.launch";

  std::ifstream ssi(ro_launcher_path.c_str());

  // Read up to (not including) the first checkpoint directive
  std::string line;
  std::vector<std::string> lines;
  while(getline(ssi, line))
  {
    if (line.find("autogeneration_checkpoint_1") == std::string::npos)
      lines.push_back(line);
    else break;
  }

  // Fill in the generated segment
  for (unsigned int i = 0; i < generated_segment.size(); i++)
    lines.push_back(generated_segment[i]);

  // Discard until the second checkpoint directive
  while(getline(ssi, line))
  {
    if (line.find("autogeneration_checkpoint_2") != std::string::npos)
      break;
  }

  // Read until the end
  while(getline(ssi, line))
    lines.push_back(line);

  ssi.close();


  // Store launcher
  std::string launcher_path = "/home/rb1/catkin_ws/src/relief-devel/launch/avanti_live_generated.launch";
  std::ofstream fil(launcher_path.c_str(), std::ios::trunc);

  if (fil.is_open())
  {
    for (unsigned int i = 0; i < lines.size(); i++)
      fil << lines[i] << std::endl;

    ROS_INFO("Done generating launcher file");
  }
  else
    ROS_ERROR("Could not store launcher");
}


void WaypointFrame::modifyYAMLOrigin(const std::string& map_name)
{
  std::string full_path =
    "/home/rb1/catkin_ws/src/relief-devel/maps/" + map_name + ".yaml";

  // Read from the yaml --------------------------------------------------------
  std::ifstream ssi(full_path.c_str());

  std::string line;
  std::vector<std::string> lines;
  while(getline(ssi, line))
  {
    if (line.find("origin") == std::string::npos)
      lines.push_back(line);
    else
      lines.push_back("origin: [0.0, 0.0, 0.0]");
  }

  ssi.close();


  // Write modified content ----------------------------------------------------
  std::ofstream sso(full_path.c_str(), std::ios::trunc);

  for (int i = 0; i < lines.size(); i++)
    sso << lines[i] << std::endl;

  sso.close();
}

} // namespace
