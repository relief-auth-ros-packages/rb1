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

#ifndef KR_RVIZ_PLUGIN_WAYPOINT_FRAME_
#define KR_RVIZ_PLUGIN_WAYPOINT_FRAME_

#ifndef Q_MOC_RUN
#include <boost/thread/mutex.hpp>
#endif

#include <stdlib.h>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <iostream>

#include <QWidget>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"

#include "ui_WaypointMapping.h"

namespace Ogre
{
class SceneNode;
class Vector3;
class SceneManager;
class Quaternion;
}

namespace rviz
{
class DisplayContext;
}

namespace interactive_markers
{
class InteractiveMarkerServer;
}

namespace Ui
{
class WaypointMappingWidget;
}

namespace waypoint_mapping_plugin
{
class WaypointMappingTool;
}

namespace waypoint_mapping_plugin
{

class WaypointFrame : public QWidget
{
  friend class WaypointMappingTool;
  Q_OBJECT

public:
  WaypointFrame(rviz::DisplayContext *context, std::map<int, Ogre::SceneNode* >* map_ptr, interactive_markers::InteractiveMarkerServer* server, QWidget *parent = 0, WaypointMappingTool* wp_tool=0);
  ~WaypointFrame();

  void enable();
  void disable();

  void setResolutionLabel();
  void getTimestampNow(std::string& ts_str);


protected:

  Ui::WaypointMappingWidget *ui_;
  rviz::DisplayContext* context_;

private Q_SLOTS:
  void startMappingButtonClicked();
  void storeMapButtonClicked();
  void resolutionChangedSlider(int r);

private:

  ros::NodeHandle nh_;
  double mapping_resolution_;
  ros::Subscriber map_sub_;


  WaypointMappingTool* wp_mapping_tool_;
  //pointers passed via contructor
  std::map<int, Ogre::SceneNode* >* sn_map_ptr_;
  Ogre::SceneManager* scene_manager_;

  interactive_markers::InteractiveMarkerServer* server_;

  //mutex for changes of variables
  boost::mutex frame_updates_mutex_;

  // Map origin
  geometry_msgs::Pose map_origin_;

  // Map subscriber callback
  void mapCallback(const nav_msgs::OccupancyGrid::Ptr& map);

  void generateLauncher(const std::string& map_name,
    const std::pair<std::string, std::string>& origin,
    const std::string& map_resolution);
  void generateLauncher2(const std::string& map_name,
    const std::pair<std::string, std::string>& origin,
    const std::string& map_resolution);

  void modifyYAMLOrigin(const std::string& map_name);
};

}

#endif
