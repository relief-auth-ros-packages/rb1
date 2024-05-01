#!/usr/bin/env python

import time
import rospy
import rfid_localisation

class RFIDLocalisation():

################################################################################
# constructor
################################################################################
  def __init__(self):

    ############################################################################
    if rospy.has_param('~callback_frequency'):
      self.callback_frequency = rospy.get_param('~callback_frequency')
    else:
      print("ERROR: callback_frequency not set; aborting")
      return

    ############################################################################
    if rospy.has_param('~rfid_locations'):
      self.rfid_locations = rospy.get_param('~rfid_locations')
    else:
      print("WARNING: rfid_locations file not set; setting default value")
      self.rfid_locations = "~/rfid_locations.txt"

    ############################################################################
    if rospy.has_param('~tags_infos'):
      self.tags_infos = rospy.get_param('~tags_infos')
    else:
      print("ERROR: tags_infos file not set; aborting")
      return

    ############################################################################
    if rospy.has_param('~robot_velocity'):
      self.robot_velocity = rospy.get_param('~robot_velocity')
    else:
      print("ERROR: robot_velocity file not set; aborting")
      return

    ############################################################################
    if rospy.has_param('~rfid_antennas_poses_filelist'):
      self.antenna_poses_filelist = \
      rospy.get_param('~rfid_antennas_poses_filelist')
    else:
      print("ERROR: rfid_antennas_poses_filelist param not set; aborting")
      return

    ############################################################################
    if rospy.has_param('~readers_antennas_polarities'):
      self.readers_antennas_polarities = rospy.get_param('~readers_antennas_polarities')
    else:
      print("ERROR: readers_antennas_polarities file not set; aborting")
      return

    ############################################################################
    if rospy.has_param('~rfid_measurements_filelist'):
      self.rfid_measurements_filelist = \
      rospy.get_param('~rfid_measurements_filelist')
    else:
      print("ERROR: rfid_measurements_filelist file not set; aborting")
      return


    ############################################################################
    # Both input and output lists
    self.tag_measurements = []
    self.reader_antenna_lamda = []
    self.tag_unwrapped_measurements = []
    self.unique_tag = []
    self.last_timestamp = -1.0;

    self.timer = \
        rospy.Timer(rospy.Duration(self.callback_frequency), self.callback)

    if False:
      print(self.callback_frequency)
      print(self.rfid_locations)
      print(self.tags_infos)
      print(self.robot_velocity)
      print(self.readers_antennas_polarities)
      print(self.antenna_poses_filelist)
      print(self.rfid_measurements_filelist)



################################################################################
# periodic callback
################################################################################
  def callback(self, timer):
    print("Entering callback")

    #try:

    start_time = time.time()

    five_list = rfid_localisation.main(\
      self.rfid_locations, \
      self.tags_infos, \
      self.robot_velocity, \
      self.readers_antennas_polarities, \
      self.antenna_poses_filelist, \
      self.rfid_measurements_filelist, \
      self.tag_measurements, \
      self.tag_unwrapped_measurements, \
      self.unique_tag, \
      self.last_timestamp, \
      self.reader_antenna_lamda)

    end_time = time.time()
    print "Tag localisation executed in", end_time-start_time, "sec"

    self.tag_measurements = five_list[0]
    self.tag_unwrapped_measurements = five_list[1]
    self.unique_tag = five_list[2]
    self.last_timestamp = five_list[3]
    self.reader_antenna_lamda = five_list[4]

#    except:
      #print "*********************************"
      #print "------- EXCEPTION CAUGHT ------- "
      #print "*********************************"


################################################################################
# main
################################################################################
if __name__ == '__main__':

  rospy.init_node('rfid_localisation_node')

  try:
    RFIDLocalisation()
    rospy.spin()
  except rospy.ROSInterruptException:
    print("SHUTTING DOWN")
    pass
