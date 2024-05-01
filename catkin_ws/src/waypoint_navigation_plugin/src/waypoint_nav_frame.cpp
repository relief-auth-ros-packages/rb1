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
/* Author: Dinesh Thakur - Modified for waypoint navigation */

#include <OGRE/OgreSceneManager.h>
#include <rviz/display_context.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "waypoint_nav_tool.h"

//#include "waypoint_nav_frame.h"
//#include "waypoint_nav_frame.h"

#include <tf/tf.h>

#include <QFileDialog>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace waypoint_nav_plugin
{

WaypointFrame::WaypointFrame(rviz::DisplayContext *context, std::map<int, Ogre::SceneNode* >* map_ptr, interactive_markers::InteractiveMarkerServer* server, int* unique_ind, QWidget *parent, WaypointNavTool* wp_tool)
  : QWidget(parent)
  , context_(context)
  , ui_(new Ui::WaypointNavigationWidget())
  , sn_map_ptr_(map_ptr)
  , unique_ind_(unique_ind)
  , server_(server)
  , frame_id_("/map")
  , output_topic_("/waypoints")
  , default_height_(0.0)
  , default_duration_(0.0)
  , default_iterations_(1)
  , selected_marker_name_("waypoint1")
  , wp_nav_tool_(wp_tool)
{
  scene_manager_ = context_->getSceneManager();

  // set up the GUI
  ui_->setupUi(this);

  wp_pub_ = nh_.advertise<nav_msgs::Path>("waypoints", 1, false);
  wp_dur_pub_ = nh_.advertise<std_msgs::Float64>("waypoints_duration", 1, true);
  wp_its_pub_ = nh_.advertise<std_msgs::Int16>("waypoints_iterations", 1, true);
  nav_pause_pub_ = nh_.advertise<std_msgs::Empty>("waypoints_pause_navigation", 1);
  nav_resume_pub_ = nh_.advertise<std_msgs::Empty>("waypoints_resume_navigation", 1);
  nav_return_home_pub_ = nh_.advertise<std_msgs::Empty>("waypoints_return_home", 1);
  nav_halt_pub_ = nh_.advertise<std_msgs::Empty>("waypoints_halt_navigation", 1);

  //connect the Qt signals and slots
  connect(ui_->publish_wp_button, SIGNAL(clicked()), this, SLOT(publishButtonClicked()));
  //connect(ui_->topic_line_edit, SIGNAL(editingFinished()), this, SLOT(topicChanged()));
  //connect(ui_->frame_line_edit, SIGNAL(editingFinished()), this, SLOT(frameChanged()));
  //connect(ui_->wp_height_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(heightChanged(double)));
  connect(ui_->wp_iterations_slider, SIGNAL(valueChanged(int)), this, SLOT(iterationsChangedSlider(int)));
  connect(ui_->wp_duration_slider, SIGNAL(valueChanged(int)), this, SLOT(durationChangedSlider(int)));
  connect(ui_->clear_all_button, SIGNAL(clicked()), this, SLOT(clearAllWaypoints()));
  connect(ui_->delete_last_wp_button, SIGNAL(clicked()), this, SLOT(deleteLastWP()));
  connect(ui_->pause_here_button, SIGNAL(clicked()), this, SLOT(pauseHereButtonClicked()));
  connect(ui_->resume_from_here_button, SIGNAL(clicked()), this, SLOT(resumeFromHereButtonClicked()));
  connect(ui_->return_home_button, SIGNAL(clicked()), this, SLOT(returnHomeButtonClicked()));
  connect(ui_->halt_navigation_button, SIGNAL(clicked()), this, SLOT(haltNavigationButtonClicked()));

  connect(ui_->x_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->y_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->yaw_slider, SIGNAL(valueChanged(int)), this, SLOT(poseChanged(int)));

  connect(ui_->save_wp_button, SIGNAL(clicked()), this, SLOT(saveButtonClicked()));
  connect(ui_->load_wp_button, SIGNAL(clicked()), this, SLOT(loadButtonClicked()));

}

WaypointFrame::~WaypointFrame()
{
  delete ui_;
  sn_map_ptr_ = NULL;
}

void WaypointFrame::enable()
{
  // activate the frame
  show();
}

void WaypointFrame::disable()
{
  wp_pub_.shutdown();
  wp_dur_pub_.shutdown();
  wp_its_pub_.shutdown();
  hide();
}

void WaypointFrame::saveButtonClicked()
{
  // Added by li9i 02/11/2021
  std::string ts_str;
  getTimestampNow(ts_str);
  std::string b_pth =
    "/home/rb1/catkin_ws/src/follow_waypoints/saved_path/waypoints_";
  std::string pth_str = b_pth + ts_str;
  const char* pth_c = pth_str.c_str();
  QString pth = QString(pth_c);

  // Original line
  //QString filename = QFileDialog::getSaveFileName(0,tr("Save Bag"),
  //"waypoints", tr("Bag Files (*.bag)"));

  QString filename = QFileDialog::getSaveFileName(0,tr("Save Waypoints File"),
    pth, tr("Bag Files (*.bag)"));

  if(filename == "")
    ROS_ERROR("No filename selected");
  else
  {
    QFileInfo info(filename);
    std::string filn = info.absolutePath().toStdString() + "/" + info.baseName().toStdString() + ".bag";
    ROS_INFO("saving waypoints to %s", filn.c_str());

    rosbag::Bag bag;
    try{
      bag.open(filn, rosbag::bagmode::Write);
    }
    catch(rosbag::BagIOException e)
    {
      ROS_ERROR("could not open bag %s", filn.c_str());
      return;
    }

    nav_msgs::Path path;

    std::map<int, Ogre::SceneNode* >::iterator sn_it;
    for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++)
    {
      Ogre::Vector3 position;
      position = sn_it->second->getPosition();

      geometry_msgs::PoseStamped pos;
      pos.pose.position.x = position.x;
      pos.pose.position.y = position.y;
      pos.pose.position.z = position.z;

      Ogre::Quaternion quat;
      quat = sn_it->second->getOrientation();
      pos.pose.orientation.x = quat.x;
      pos.pose.orientation.y = quat.y;
      pos.pose.orientation.z = quat.z;
      pos.pose.orientation.w = quat.w;

      path.poses.push_back(pos);
    }

    path.header.frame_id = frame_id_.toStdString();

    // Save waypoints
    bag.write("waypoints", ros::Time::now(), path);

    // Save duration
    std_msgs::Float64 dur;
    dur.data = default_duration_;
    bag.write("waypoints_duration", ros::Time::now(), dur);

    // Save iterations
    std_msgs::Int16 its;
    its.data = default_iterations_;
    bag.write("waypoints_iterations", ros::Time::now(), its);

    bag.close();
    ROS_INFO("saved waypoints to %s", filn.c_str());
  }
}

void WaypointFrame::loadButtonClicked()
{
  QString filename = QFileDialog::getOpenFileName(0,tr("Open Waypoints File"),
    "/home/rb1/catkin_ws/src/follow_waypoints/saved_path/",
    tr("Bag Files (*.bag)"));

  if(filename == "")
    ROS_ERROR("No filename selected");
  else
  {
    //Clear existing waypoints
    clearAllWaypoints();

    std::string filn = filename.toStdString();
    ROS_INFO("loading waypoints from %s", filn.c_str());

    rosbag::Bag bag;
    try{
      bag.open(filn, rosbag::bagmode::Read);
    }
    catch(rosbag::BagIOException e)
    {
      ROS_ERROR("could not open bag %s", filn.c_str());
      return;
    }

    std::vector<std::string> topics;
    topics.push_back(std::string("waypoints"));
    rosbag::View view_p(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view_p)
    {
      nav_msgs::Path::ConstPtr p = m.instantiate<nav_msgs::Path>();
      if (p != NULL)
      {
        ROS_INFO("n waypoints: %zu", p->poses.size());

        for(int i = 0; i < p->poses.size(); i++)
        {
          geometry_msgs::PoseStamped pos = p->poses[i];
          Ogre::Vector3 position;
          position.x = pos.pose.position.x;
          position.y = pos.pose.position.y;
          position.z = pos.pose.position.z;

          Ogre::Quaternion quat;
          quat.x = pos.pose.orientation.x;
          quat.y = pos.pose.orientation.y;
          quat.z = pos.pose.orientation.z;
          quat.w = pos.pose.orientation.w;

          wp_nav_tool_->makeIm(position, quat);
        }
      }
    }


    topics.clear();
    topics.push_back(std::string("waypoints_duration"));
    rosbag::View view_d(bag, rosbag::TopicQuery(topics));

    double loaded_duration;
    int loaded_iterations;

    foreach(rosbag::MessageInstance const m, view_d)
    {
      std_msgs::Float64::ConstPtr p = m.instantiate<std_msgs::Float64>();
      if (p != NULL)
      {
        ROS_INFO("duration (h): %f", p->data);
        loaded_duration = p->data;
      }
    }

    topics.clear();
    topics.push_back(std::string("waypoints_iterations"));
    rosbag::View view_i(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view_i)
    {
      std_msgs::Int16::ConstPtr p = m.instantiate<std_msgs::Int16>();
      if (p != NULL)
      {
        ROS_INFO("iterations: %d", p->data);
        loaded_iterations = p->data;
      }
    }


    setConfig(loaded_iterations, loaded_duration);
  }
}

void WaypointFrame::publishButtonClicked()
{
  nav_msgs::Path path;

  std::map<int, Ogre::SceneNode* >::iterator sn_it;
  for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++)
  {
    Ogre::Vector3 position;
    position = sn_it->second->getPosition();

    geometry_msgs::PoseStamped pos;
    pos.pose.position.x = position.x;
    pos.pose.position.y = position.y;
    pos.pose.position.z = position.z;

    Ogre::Quaternion quat;
    quat = sn_it->second->getOrientation();
    pos.pose.orientation.x = quat.x;
    pos.pose.orientation.y = quat.y;
    pos.pose.orientation.z = quat.z;
    pos.pose.orientation.w = quat.w;

    path.poses.push_back(pos);
  }

  path.header.frame_id = frame_id_.toStdString();
  wp_pub_.publish(path);

  // Publish the duration and iterations as well
  std_msgs::Float64 dur;
  dur.data = default_duration_;
  wp_dur_pub_.publish(dur);

  std_msgs::Int16 its;
  its.data = default_iterations_;
  wp_its_pub_.publish(its);
}

void WaypointFrame::clearAllWaypoints()
{
  //destroy the ogre scene nodes
  std::map<int, Ogre::SceneNode* >::iterator sn_it;
  for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++)
  {
    scene_manager_->destroySceneNode(sn_it->second);
  }

  //clear the waypoint map and reset index
  sn_map_ptr_->clear();
  *unique_ind_=0;

  //clear the interactive markers
  server_->clear();
  server_->applyChanges();
}

void WaypointFrame::deleteLastWP()
{
  --*unique_ind_;
  std::map<int, Ogre::SceneNode* >::iterator sn_it = --sn_map_ptr_->end();
  sn_it->second->detachAllObjects();
  std::stringstream wp_name;
  wp_name << "waypoint" << sn_it->first;
  std::string wp_name_str(wp_name.str());
  server_->erase(wp_name_str);
  server_->applyChanges();
  sn_map_ptr_->erase(sn_it);

}

void WaypointFrame::pauseHereButtonClicked()
{
  std_msgs::Empty e;
  nav_pause_pub_.publish(e);
}
void WaypointFrame::resumeFromHereButtonClicked()
{
  std_msgs::Empty e;
  nav_resume_pub_.publish(e);
}

void WaypointFrame::returnHomeButtonClicked()
{
  std_msgs::Empty e;
  nav_return_home_pub_.publish(e);
}

void WaypointFrame::haltNavigationButtonClicked()
{
  std_msgs::Empty e;
  nav_halt_pub_.publish(e);
}


/*
   void WaypointFrame::heightChanged(double h)
   {
   boost::mutex::scoped_lock lock(frame_updates_mutex_);
   default_height_ = h;
   }
   */

void WaypointFrame::iterationsChangedSlider(int i)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  default_iterations_ = i;

  setWpIterationsLabel();
}

void WaypointFrame::durationChangedSlider(int d)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  default_duration_ = static_cast<double>(d)/2;

  setWpDurationLabel();
}

void WaypointFrame::setSelectedMarkerName(std::string name)
{
  selected_marker_name_ = name;
}

void WaypointFrame::poseChanged(double val)
{
  std::map<int, Ogre::SceneNode *>::iterator sn_entry =
    sn_map_ptr_->find(std::stoi(selected_marker_name_.substr(8)));

  if (sn_entry == sn_map_ptr_->end())
    ROS_ERROR("%s not found in map", selected_marker_name_.c_str());
  else
  {
    Ogre::Vector3 position;
    Ogre::Quaternion quat;
    getPose(position, quat);

    sn_entry->second->setPosition(position);
    sn_entry->second->setOrientation(quat);

    std::stringstream wp_name;
    wp_name << "waypoint" << sn_entry->first;
    std::string wp_name_str(wp_name.str());

    visualization_msgs::InteractiveMarker int_marker;
    if(server_->get(wp_name_str, int_marker))
    {
      int_marker.pose.position.x = position.x;
      int_marker.pose.position.y = position.y;
      int_marker.pose.position.z = position.z;

      int_marker.pose.orientation.x = quat.x;
      int_marker.pose.orientation.y = quat.y;
      int_marker.pose.orientation.z = quat.z;
      int_marker.pose.orientation.w = quat.w;

      server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
    }
    server_->applyChanges();
  }
}

void WaypointFrame::poseChanged(int val)
{
  std::ostringstream stringStream;
  stringStream << val;
  std::string st = stringStream.str();

  ui_->yaw_slider_value_label->blockSignals(true);
  ui_->yaw_slider_value_label->setText(QString::fromStdString(st));
  ui_->yaw_slider_value_label->blockSignals(false);


  std::map<int, Ogre::SceneNode *>::iterator sn_entry =
    sn_map_ptr_->find(std::stoi(selected_marker_name_.substr(8)));

  if (sn_entry == sn_map_ptr_->end())
    ROS_ERROR("%s not found in map", selected_marker_name_.c_str());
  else
  {
    Ogre::Vector3 position;
    Ogre::Quaternion quat;
    getPose(position, quat);

    sn_entry->second->setPosition(position);
    sn_entry->second->setOrientation(quat);

    std::stringstream wp_name;
    wp_name << "waypoint" << sn_entry->first;
    std::string wp_name_str(wp_name.str());

    visualization_msgs::InteractiveMarker int_marker;
    if(server_->get(wp_name_str, int_marker))
    {
      int_marker.pose.position.x = position.x;
      int_marker.pose.position.y = position.y;
      int_marker.pose.position.z = position.z;

      int_marker.pose.orientation.x = quat.x;
      int_marker.pose.orientation.y = quat.y;
      int_marker.pose.orientation.z = quat.z;
      int_marker.pose.orientation.w = quat.w;

      server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
    }
    server_->applyChanges();
  }
}

/*
   void WaypointFrame::frameChanged()
   {

   boost::mutex::scoped_lock lock(frame_updates_mutex_);
   QString new_frame = ui_->frame_line_edit->text();

// Only take action if the frame has changed.
if((new_frame != frame_id_)  && (new_frame != ""))
{
frame_id_ = new_frame;
ROS_INFO("new frame: %s", frame_id_.toStdString().c_str());

// update the frames for all interactive markers
std::map<int, Ogre::SceneNode *>::iterator sn_it;
for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++)
{
std::stringstream wp_name;
wp_name << "waypoint" << sn_it->first;
std::string wp_name_str(wp_name.str());

visualization_msgs::InteractiveMarker int_marker;
if(server_->get(wp_name_str, int_marker))
{
int_marker.header.frame_id = new_frame.toStdString();
server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
}
}
server_->applyChanges();
}
}
*/

/*
   void WaypointFrame::topicChanged()
   {
   QString new_topic = ui_->topic_line_edit->text();

// Only take action if the name has changed.
if(new_topic != output_topic_)
{
wp_pub_.shutdown();
output_topic_ = new_topic;

if((output_topic_ != "") && (output_topic_ != "/"))
{
wp_pub_ = nh_.advertise<nav_msgs::Path>(output_topic_.toStdString(), 1);
}
}
}
*/

void WaypointFrame::setWpCount(int size)
{
  std::ostringstream stringStream;
  stringStream << "num wp: " << size;
  std::string st = stringStream.str();

  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  ui_->waypoint_count_label->setText(QString::fromStdString(st));
}

void WaypointFrame::setWpDurationSlider(double d)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  ui_->wp_duration_slider->blockSignals(true);
  ui_->wp_duration_slider->setValue(d);
  ui_->wp_duration_slider->blockSignals(false);
}

void WaypointFrame::setWpDurationLabel()
{
  std::ostringstream stringStream;
  stringStream << default_duration_;
  std::string st = stringStream.str();

  ui_->wp_duration_value_label->blockSignals(true);
  ui_->wp_duration_value_label->setText(QString::fromStdString(st));
  ui_->wp_duration_value_label->blockSignals(false);
}

void WaypointFrame::setWpIterationsSlider(int i)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  ui_->wp_iterations_slider->blockSignals(true);
  ui_->wp_iterations_slider->setValue(i);
  ui_->wp_iterations_slider->blockSignals(false);
}

void WaypointFrame::setWpIterationsLabel()
{
  std::ostringstream stringStream;
  stringStream << default_iterations_;
  std::string st = stringStream.str();

  ui_->wp_iterations_value_label->blockSignals(true);
  ui_->wp_iterations_value_label->setText(QString::fromStdString(st));
  ui_->wp_iterations_value_label->blockSignals(false);
}



/*
   void WaypointFrame::setConfig(QString topic, QString frame, float height)
   {
   {
   boost::mutex::scoped_lock lock(frame_updates_mutex_);
   ui_->topic_line_edit->blockSignals(true);
   ui_->frame_line_edit->blockSignals(true);
   ui_->wp_height_doubleSpinBox->blockSignals(true);

   ui_->topic_line_edit->setText(topic);
   ui_->frame_line_edit->setText(frame);
   ui_->wp_height_doubleSpinBox->setValue(height);

   ui_->topic_line_edit->blockSignals(false);
   ui_->frame_line_edit->blockSignals(false);
   ui_->wp_height_doubleSpinBox->blockSignals(false);

   }
   topicChanged();
   frameChanged();
   heightChanged(height);
   }
   */

void WaypointFrame::setConfig(int iterations, float duration)
{
  {
    setWpIterationsSlider(iterations);
    setWpDurationSlider(2*duration);
  }
  iterationsChangedSlider(iterations);
  durationChangedSlider(2*duration);
}


void WaypointFrame::getPose(Ogre::Vector3& position, Ogre::Quaternion& quat)
{
  {
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
    position.x = ui_->x_doubleSpinBox->value();
    position.y = ui_->y_doubleSpinBox->value();
    position.z = default_height_;
    double yaw = 2*M_PI/360 * ui_->yaw_slider->value();

    tf::Quaternion qt = tf::createQuaternionFromYaw(yaw);
    quat.x = qt.x();
    quat.y = qt.y();
    quat.z = qt.z();
    quat.w = qt.w();
  }
}

void WaypointFrame::setPose(Ogre::Vector3& position, Ogre::Quaternion& quat)
{
  {
    //boost::mutex::scoped_lock lock(frame_updates_mutex_);
    //block spinbox signals
    ui_->x_doubleSpinBox->blockSignals(true);
    ui_->y_doubleSpinBox->blockSignals(true);
    ui_->yaw_slider->blockSignals(true);

    ui_->x_doubleSpinBox->setValue(position.x);
    ui_->y_doubleSpinBox->setValue(position.y);

    tf::Quaternion qt(quat.x, quat.y, quat.z, quat.w);
    ui_->yaw_slider->setValue(static_cast<int>(tf::getYaw(qt)*360 / (2*M_PI)));

    //enable the signals
    ui_->x_doubleSpinBox->blockSignals(false);
    ui_->y_doubleSpinBox->blockSignals(false);
    ui_->yaw_slider->blockSignals(false);

  }
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

void WaypointFrame::setWpLabel(Ogre::Vector3 position)
{
  {
    //boost::mutex::scoped_lock lock(frame_updates_mutex_);
    std::ostringstream stringStream;
    stringStream.precision(2);
    stringStream << selected_marker_name_;
    //stringStream << " x: " << position.x << " y: " << position.y << " z: " << position.z;
    std::string label = stringStream.str();

    ui_->sel_wp_label->setText(QString::fromStdString(label));
  }
}

double WaypointFrame::getDefaultHeight()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return default_height_;
}

QString WaypointFrame::getFrameId()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return frame_id_;
}

QString WaypointFrame::getOutputTopic()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return output_topic_;
}

double WaypointFrame::getDefaultDuration()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return default_duration_;
}

int WaypointFrame::getDefaultIterations()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return default_iterations_;
}
} // namespace
