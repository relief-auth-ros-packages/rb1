#!/usr/bin/env python

import rospy
import yaml
import rospkg
import tf
from poi_manager.msg import *
from poi_manager.srv import *
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Empty
from visualization_msgs.msg import MarkerArray, Marker

class ManageYAML:

    def __init__(self):
        rospack = rospkg.RosPack()
        self.filename = rospy.get_param('~filename', 'test')
        self.yaml_path = rospack.get_path('poi_manager') +'/'+ self.filename+'.yaml'
        self.pose_list = []
        self.pose_dict = {}
        self.service_read_yaml = rospy.Service('~read_pois', ReadPOIs, self.handle_labeled_pose_list)
        self.service_write_data = rospy.Service('~update_pois', UpdatePOIs, self.handle_updated_list)
        
        self.publish_markers = rospy.get_param('~publish_markers',False)
        if self.publish_markers:
            self.marker_array = MarkerArray()
            self.marker_topic = rospy.get_param('~marker_topic', 'markers')
            self.frequency = rospy.get_param('~frequency', 0.5)
            self.frame_id = rospy.get_param('~frame_id', 'map')
            self.marker_array_publisher = rospy.Publisher(self.filename+'/'+self.marker_topic, MarkerArray, queue_size=10)


    def start(self):
        while not rospy.is_shutdown() and self.publish_markers:
            self.marker_array_publisher.publish(self.marker_array)
            rospy.sleep(1/self.frequency)
            
    def parse_yaml(self):
        try:
            f = open(self.yaml_path, 'r')
            self.pose_dict = {}
            self.pose_dict = yaml.safe_load(f)
            if self.pose_dict is None:
                self.pose_dict = {}
            f.close()
        except (IOError, yaml.YAMLError) as e:
        	rospy.logerr(e)
        	return 0

    def manage_read_data(self):
        self.pose_list = []
        for key, value in self.pose_dict.items():
            self.pose_list.append(LabeledPose(key, Pose2D(value[0], value[1], value[2])))

        if self.publish_markers:
            self.update_marker_array()


    def update_yaml(self, req):
        yaml_file = file(self.yaml_path, 'w')
        pose_list = req.pose_list
        self.pose_dict = {}
        for elem in pose_list:
            self.pose_dict[elem.label] = [elem.pose.x, elem.pose.y, elem.pose.theta]
        yaml.dump(self.pose_dict, yaml_file)

        if self.publish_markers:
            self.update_marker_array()

    def update_marker_array(self):
        marker_array = MarkerArray()
        for item in self.pose_dict.items():
            # Two markers are created for each item
            # The first one shows the pose
            marker_arrow = self.create_marker(item, 'arrow')
            # The second one shows the label
            marker_text = self.create_marker(item, 'text')
            marker_array.markers.append(marker_arrow)
            marker_array.markers.append(marker_text)
        self.marker_array = marker_array
        self.marker_array_publisher.publish(self.marker_array)

    def create_marker(self, data, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time()
        marker.ns = data[0]
        marker.action = Marker.ADD
        marker.pose.position.x = data[1][0]
        marker.pose.position.y = data[1][1]
        quaternion = tf.transformations.quaternion_from_euler(0, 0, data[1][2])
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        marker.scale.x = 1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        # TODO parameterize arrow color
        marker.color.a = 1.0 
        marker.color.g = 1.0
        marker.lifetime = rospy.Duration(1/self.frequency)
        if marker_type == 'arrow':
            marker.id = 0
            marker.type = Marker.ARROW
        else:
            marker.id = 1
            marker.type = Marker.TEXT_VIEW_FACING
            marker.text = data[0]
            marker.pose.position.z = 0.5
            marker.color.r = 1.0
            marker.color.b = 1.0 
        return marker

    def handle_labeled_pose_list(self, req):
        self.parse_yaml()
        self.manage_read_data()
        print "read_pois service done"
        return ReadPOIsResponse(self.pose_list)

    def handle_updated_list(self, req):
        self.update_yaml(req)
        print "update_pois service done"
        return UpdatePOIsResponse()
    

def main():
    rospy.init_node('manage_yaml')
    yaml_manager = ManageYAML()
    yaml_manager.start()
    print "Node running."
    rospy.spin()


if __name__ == "__main__":
	main()
