#ifndef RFID_VISUALISATION_H
#define RFID_VISUALISATION_H

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <set>
#include <cstdlib>
#include <regex>

#include "ros/ros.h"
#include "ros/master.h"
#include "visualization_msgs/Marker.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cv_bridge/cv_bridge.h>
#include <rviz_plugin_manager/PluginLoad.h>
#include <rviz_plugin_manager/PluginUnload.h>
#include <rviz_plugin_manager/PluginSetConfig.h>
#include <dynamic_reconfigure/server.h>
#include <relief_rfid_visualisation/relief_rfid_visualisation_cfgConfig.h>
#include "std_msgs/Empty.h"
#include <relief_rfid_visualisation/ListInventory.h>
#include <relief_rfid_visualisation/VisualiseAll.h>
#include <relief_rfid_visualisation/VisualiseNone.h>
#include <relief_rfid_visualisation/VisualiseThisEPC.h>
#include <relief_rfid_visualisation/VisualiseThisType.h>


class RfidVisualisation
{
  private:
    ros::NodeHandle nodehandle_;

    // The dynamic reconfigure parameters' server
    dynamic_reconfigure::Server<
      relief_rfid_visualisation::relief_rfid_visualisation_cfgConfig> dr_server_;

    // List topics
    std::string list_inventory_topic_;
    std::string visualise_all_tags_topic_;
    std::string visualise_no_tags_topic_;
    std::string visualise_this_tag_topic_;
    std::string visualise_this_tag_type_topic_;

    // List services
    ros::ServiceServer list_inventory_srv_;
    ros::ServiceServer visualise_all_epcs_srv_;
    ros::ServiceServer visualise_no_epcs_srv_;
    ros::ServiceServer visualise_this_epc_srv_;
    ros::ServiceServer visualise_this_type_srv_;

    // List Marker publisher
    ros::Publisher marker_pub_;
    ros::Publisher bookcase_marker_pub_;
    ros::Publisher hanger_marker_pub_;

    // the coordinates of bookcase corner points (4) per bookcase
    std::vector< std::pair<double,double> > bookcase_corner_coords_;

    // the coordinates of hanger points (3) per hanger
    std::vector< std::pair<double,double> > hanger_coords_;

    // This vector holds categories of epc's (the second column in tag_info.txt)
    // whose height should be decreased due to high antenna placement
    std::vector<std::string> decrease_height_categories_;

    // This vector holds the height by which the above categories should be
    // decreased
    std::vector<double> decrease_height_categories_height_;

    // Indicates whether images are used for visualisation or markers
    bool use_images_;

    // If a service constrains the view this variable turns true
    bool view_locked_;

    // If a service is being executed prevent from executing another one at the
    // same time
    bool service_lock_;

    // For changing between images and markers but showing the same types or
    // EPC without resetting
    std::string latest_type_requested_;
    std::string latest_epc_requested_;
    unsigned int latest_service_requested_;

    // List logfile filenames
    std::string rfid_locations_filename_;
    std::string rfid_tag_info_filename_;
    std::string rfid_tag_images_directory_;
    std::string inventory_filename_;

    // The timer
    ros::WallTimer periodic_timer_;

    // Callback period (s).
    // Markers are published every `callback_period_` seconds.
    double callback_period_;

    // A map where: EPC --> type (e.g. book), image topic
    std::unordered_map<std::string, std::vector<std::string> > tag_info_map_;

    // A map where: EPC --> plugin_uid (rviz_plugin_manager)
    std::unordered_map<std::string, unsigned int> tag_plugin_uid_map_;

    // List callbacks
    void parametersDynamicCallback(
      const relief_rfid_visualisation::relief_rfid_visualisation_cfgConfig& config,
      const uint32_t& level);
    void periodicCallback(const ros::WallTimerEvent& event);

    // Init / helpers
    int callRvizPluginManagerConfigSetService(const int& plugin_uid,
      const std::string& config_string);
    int callRvizPluginManagerPluginLoadService(
      const std::string& plugin_class,
      const std::string& plugin_name,
      const std::string& plugin_topic,
      const std::string& plugin_data_type,
      const std::string& plugin_config,
      int *plugin_uid);

    int convertEPCToInt(const std::string& EPC);
    bool decreaseTagHeight(const std::string& epc,
      const std::vector<std::string>& categories);
    void deleteAllMarkers();
    void findTagTypesTopics(std::unordered_map<std::string, std::vector<std::string> >* tag_info_map);
    std::set<std::string> getCategoriesFromTagInfo();
    std::vector<std::string> getEpcsFromResultsFile();
    std::vector<unsigned int> getNumberOfItemsFoundPerCategory(
      const std::vector<std::string>& found_epcs,
      const std::set<std::string>& all_categories);
    std::vector<unsigned int> getTotalNumberOfItemsPerCategory(
      const std::set<std::string>& categories);
    std::unordered_map<std::string, std::pair<unsigned int, unsigned int> > listInventory();
    bool listInventorySRV(
      relief_rfid_visualisation::ListInventory::Request& req,
      relief_rfid_visualisation::ListInventory::Response& res);
    void loadParams();
    void loadTagInfo(std::unordered_map<std::string, std::vector<std::string> >* map);
    void publishBookcaseMarkers();
    void publishHangerMarkers();
    void publishEPCFrame(const std::vector<std::string>& info);
    int publishEPCRvizPlugin(const std::vector<std::string>& info);
    void publishImage(const std::vector<std::string>& marker_info);
    void publishMarker(const std::vector<std::string>& marker_info);
    void publishMarkerShape(const std::vector<std::string>& marker_info);
    void publishMarkerText(const std::vector<std::string>& marker_info);
    void publishTagTypesImages(
      const std::string& images_directory,
      const std::unordered_map<std::string, std::vector<std::string> >& tag_info_map);

    void visualiseAllEPCs();
    bool visualiseAllEPCsSRV(
      relief_rfid_visualisation::VisualiseAll::Request& req,
      relief_rfid_visualisation::VisualiseAll::Response& res);
    void visualiseNoEPCs();
    bool visualiseNoEPCsSRV(
      relief_rfid_visualisation::VisualiseNone::Request& req,
      relief_rfid_visualisation::VisualiseNone::Response& res);
    void visualiseThisEPC(const std::string& EPC);
    bool visualiseThisEPCSRV(
      relief_rfid_visualisation::VisualiseThisEPC::Request& req,
      relief_rfid_visualisation::VisualiseThisEPC::Response& res);
    void visualiseThisType(const std::string& type);
    bool visualiseThisTypeSRV(
      relief_rfid_visualisation::VisualiseThisType::Request& req,
      relief_rfid_visualisation::VisualiseThisType::Response& res);
    void writeInventoryToFile();



  public:
    RfidVisualisation(void);
    ~RfidVisualisation(void);
};

#endif // RFID_VISUALISATION_H
