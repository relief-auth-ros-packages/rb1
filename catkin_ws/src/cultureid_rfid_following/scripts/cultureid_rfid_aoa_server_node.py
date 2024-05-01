#!/usr/bin/env python
# http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

import time
import rospy
from std_msgs.msg import Empty
from cultureid_rfid_following.msg import AoAMsg
from cultureid_rfid_following.srv import AoASrv, AoASrvResponse

from tta import *
from tfa import *

class CultureidRFIDAoA():

################################################################################
# constructor
################################################################################
  def __init__(self):

    if rospy.has_param('~rfid_measurements_file'):
      self.rfid_measurements_file = rospy.get_param('~rfid_measurements_file')
    else:
      print("ERROR: rfid_measurements_file not set; aborting")
      return

    if rospy.has_param('~bank_rfid_measurements_file'):
      self.bank_rfid_measurements_file = rospy.get_param('~bank_rfid_measurements_file')
    else:
      print("ERROR: bank_rfid_measurements_file not set; aborting")
      return

    if rospy.has_param('~epc'):
      self.epc = rospy.get_param('~epc')
    else:
      print("ERROR: EPC not set; aborting")
      return

    if rospy.has_param('~request_lost_tag_topic'):
      self.request_lost_tag_topic = rospy.get_param('~request_lost_tag_topic')
    else:
      print("ERROR: request_lost_tag_topic not set; aborting")
      return

    if rospy.has_param('~lost_tag_detected_topic'):
      self.lost_tag_detected_topic = rospy.get_param('~lost_tag_detected_topic')
    else:
      print("ERROR: lost_tag_detected_topic not set; aborting")
      return




################################################################################
# this is where the aoa is calculated
################################################################################
  def aoaFunction(self, req):

    #a = track(self.rfid_measurements_file, self.epc, req.past_aoas[0].aoa[0])

    #print(req)


#    t1 = req.past_aoas[0].aoa[0]
    #x1_l = req.past_aoas[0].aoa[5]
    #y1_l = req.past_aoas[0].aoa[6]
    #x1_r = req.past_aoas[0].aoa[1]
    #y1_r = req.past_aoas[0].aoa[2]

    #t2 = req.past_aoas[1].aoa[0]
    #x2_l = req.past_aoas[1].aoa[5]
    #y2_l = req.past_aoas[1].aoa[6]
    #x2_r = req.past_aoas[1].aoa[1]
    #y2_r = req.past_aoas[1].aoa[2]

    #poses_input = [t1, x1_l, y1_l, x1_r, y1_r, t2, x2_l, y2_l, x2_r, y2_r]

    #last_estimation = req.past_aoas[2].aoa[0]
    #past_estimation = req.past_aoas[2].aoa[1]

    # moving_main
    #a = track(self.rfid_measurements_file, self.bank_rfid_measurements_file, \
      #self.epc, last_estimation, past_estimation, poses_input)

    #a = track(self.rfid_measurements_file, \
      #self.epc, last_estimation, poses_input)

    # pf
    #a = track(self.rfid_measurements_file, self.epc, last_estimation,\
      #100, 0.1, 0.3, 10)

    # tta
    last_estimation = req.past_aoas[2].aoa[0]
    a = track(self.rfid_measurements_file, self.epc, last_estimation,\
      100, 0.1, 0.3, 10)


    z = AoAMsg()

    # moving main
    z.aoa.append(a[0])
    z.aoa.append(a[1])
    z.aoa.append(a[2])

    # moving main
    #z.aoa.append(a[0])
    #z.aoa.append(a[1])
    #z.aoa.append(a[2])
    #z.aoa.append(a[3])

    # pf
    #z.aoa.append(a[0])
    #z.aoa.append(a[1])

    # Dummy
    #z.aoa.append(0.0)
    #z.aoa.append(1.0)

    return AoASrvResponse(z)


################################################################################
# when the tag is lost, request it be found
################################################################################
  def request_lost_tag_detection_callback(self, data):

    print("[python] requesting lost tag detection")
    target_finder(self.rfid_measurements_file, self.epc)

    mt_msg = Empty()
    lost_tag_detected_pub.publish(mt_msg)
    print("[python] sent 'lost tag detected' msg")

    return


################################################################################
# main
################################################################################
if __name__ == '__main__':

  rospy.init_node('cultureid_rfid_aoa_server')

  try:
    c = CultureidRFIDAoA()
    s = rospy.Service(rospy.get_name()+'_aoa_srv', AoASrv, c.aoaFunction)

    # listens for messages from the c++ node
    request_lost_tag_detection_sub = \
      rospy.Subscriber(c.request_lost_tag_topic, Empty, c.request_lost_tag_detection_callback)

    # Publishes an autorotation cease message when the tag it found again
    lost_tag_detected_pub = \
      rospy.Publisher(c.lost_tag_detected_topic, Empty, queue_size=10)

    rospy.spin()
  except rospy.ROSInterruptException:
    print("SHUTTING DOWN")
    pass
