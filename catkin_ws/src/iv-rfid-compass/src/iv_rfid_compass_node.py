#!/usr/bin/env python

import numpy as np
import math
import time
import shutil
import mmap
from itertools import islice
import rospy


################################################################################
# Tag name is a class to identify the tag number id from the tags serial number
################################################################################
class Tag_name():
  def __init__(self, id, serial_number):
    self.id = id
    self.serial_number = serial_number

  def set_id(self, id):
    self.id = id
  def set_serial_number(self, serial_number):
    self.serial_number = serial_number

  def get_id(self):
    return self.id
  def get_serial_number(self):
    return self.serial_number


################################################################################
# Tag class: id,serial number,time,phase,slope
################################################################################
class Tag():
  def __init__(self, tag_id, tag_serial_number, time, wrapped_phase, unwrapped_phase, slope):
    self.time = time
    self.tag_id = tag_id
    self.tag_serial_number = tag_serial_number
    self.wrapped_phase = wrapped_phase
    self.unwrapped_phase = unwrapped_phase
    self.slope = slope

  def set_time(self, time):
    self.time = time
  def set_tag_id(self, tag_id):
    self.tag_id = tag_id
  def set_tag_serial_number(self, tag_serial_number):
    self.tag_serial_number = tag_serial_number
  def set_wrapped_phase(self, wrapped_phase):
    self.wrapped_phase = wrapped_phase
  def set_unwrapped_phase(self, unwrapped_phase):
    self.unwrapped_phase = unwrapped_phase
  def set_slope(self, slope):
    self.slope = slope

  def get_time(self):
    return self.time
  def get_tag_id(self):
    return self.tag_id
  def get_tag_serial_number(self):
    return self.tag_serial_number
  def get_wrapped_phase(self):
    return self.wrapped_phase
  def get_unwrapped_phase(self):
    return self.unwrapped_phase
  def get_slope(self):
    return self.slope


################################################################################
# This is the class where the magic happens
################################################################################
class iv():

  #-----------------------------------------------------------------------------
  def __init__(self):

    if rospy.has_param('~rfid_measurements_file'):
      self.rfid_measurements_file = rospy.get_param('~rfid_measurements_file')

      rospy.loginfo('[iv_rfid_compass] Resetting %s file', self.rfid_measurements_file)
      open(self.rfid_measurements_file, 'w').close()
    else:
      rospy.logerr("[iv_rfid_compass] ERROR: rfid_measurements_file not set; aborting")
      return

    if rospy.has_param('~rfid_measurements_file_readonly'):
      self.rfid_measurements_file_readonly = rospy.get_param('~rfid_measurements_file_readonly')

      rospy.loginfo('[iv_rfid_compass] Resetting %s file', self.rfid_measurements_file_readonly)
      open(self.rfid_measurements_file_readonly, 'w').close()
    else:
      rospy.logerr("[iv_rfid_compass] ERROR: rfid_measurements_file_readonly not set; aborting")
      return

    if rospy.has_param('~rfid_measurements_file_working_copy'):
      self.rfid_measurements_file_working_copy = rospy.get_param('~rfid_measurements_file_working_copy')

      rospy.loginfo('[iv_rfid_compass] Resetting %s file', self.rfid_measurements_file_working_copy)
      open(self.rfid_measurements_file_working_copy, 'w').close()
    else:
      rospy.logerr("[iv_rfid_compass] ERROR: rfid_measurements_file_working_copy not set; aborting")
      return

    if rospy.has_param('~tag_enumeration_file'):
      self.tag_enumeration_file = rospy.get_param('~tag_enumeration_file')
    else:
      rospy.logerr("[iv_rfid_compass] ERROR: tag_enumeration_file not set; aborting")
      return

    if rospy.has_param('~tag_to_seek'):
      self.tag_to_seek = rospy.get_param('~tag_to_seek')
    else:
      rospy.logerr("[iv_rfid_compass] ERROR: tag_to_seek not set; aborting")
      return

    if rospy.has_param('~node_decisions'):
      self.node_decisions = rospy.get_param('~node_decisions')

      rospy.loginfo('[iv_rfid_compass] Resetting %s file', self.node_decisions)
      open(self.node_decisions, 'w').close()
    else:
      rospy.logerr("[iv_rfid_compass] ERROR: node_decisions not set; aborting")
      return

    if rospy.has_param('~callback_period'):
      self.callback_period = rospy.get_param('~callback_period')
    else:
      rospy.logerr("[iv_rfid_compass] ERROR: callback_period not set; aborting")
      return

    if rospy.has_param('~minimum_slope'):
      self.minimum_slope = rospy.get_param('~minimum_slope')
    else:
      rospy.logerr("[iv_rfid_compass] ERROR: minimum_slope not set; aborting")
      return

    # List to save the data from Tag_name class
    self.tag_name = []

    # List to save the wrapped phase of the current read
    self.wrapped_phase_list = []

    # List to save the unwrapped phase
    self.unwrapped_phase_list = []

    # List to save the data from Tag class
    self.tag_list = []

    # open serial number to id text file
    with open(self.tag_enumeration_file, 'r') as read_file:
      for line in read_file:
        # tag_name is a list to save the data from Tag_name class
        # Important!! Careful of the way it is writen on the txt file.
        # Example, if there is a space before the serial number!!
        self.tag_name.append(\
            Tag_name(int(line.split(',')[0]), line.split(',')[1]))

        # We need as many wrapped phase lists as the number of tags
        self.wrapped_phase_list.append([])

        # We need as many tag_list lists as the number of tags
        self.tag_list.append([])


    # Call `self.continuous_loop` every `self.callback_period` seconds
    self.iv_timer = rospy.Timer(\
        rospy.Duration(self.callback_period), self.continuous_loop)

    rospy.loginfo('[iv_rfid_compass] Init completed')


  #-----------------------------------------------------------------------------
  def clear_structures(self):

    for i in range(len(self.tag_list)):
      self.tag_list[i] = []
    for i in range(len(self.wrapped_phase_list)):
      self.wrapped_phase_list[i] = []


  #-----------------------------------------------------------------------------
  def continuous_loop(self, timer):

    cl_t1 = time.time()

    # 0. Open main rfid measurements file.
    elm_t1 = time.time()
    self.process_latest_measurements()
    elm_t2 = time.time()

    rospy.loginfo('[iv_rfid_compass] -----------------------------------')
    rospy.loginfo('[iv_rfid_compass] Measurements handled in %f ms', 1000*(elm_t2-elm_t1))

    # 1. Open relut txt file
    with open(self.rfid_measurements_file_working_copy, 'r') as read_file:

      # first_time saves the time from the first measurement of the each txt file
      for line in read_file:
        first_time = line.split(',')[0]
        break # Mporei na ginetai me kalitero tropo den mporoysa na skefto kati allo tora...

      for line in read_file:
        for index_tag_id in range(len(self.tag_name)):

          # 2. Find the correct tag id name from the serial number that we read from the txt file
          if(line.split(', ')[2] == self.tag_name[index_tag_id].get_serial_number()[:-1]):

            # id_variable is the correct id
            id_variable = self.tag_name[index_tag_id].get_id()

            # Save the wrapped phase that we read from the txt file
            self.wrapped_phase_list[index_tag_id].append(-float(line.split(',')[3]))

            # Unwrap that phase and save it to the
            # unwrapped_phase_list list
            self.unwrapped_phase_list = np.unwrap(self.wrapped_phase_list[index_tag_id])

            # 3. Then create a new object and save it in the
            # correct list that comes from the id Tag(tag_id,
            # tag_serial_number, time, wrapped_phase, unwrapped_phase, slope)
            self.tag_list[index_tag_id].append(\
                Tag(id_variable, line.split(',')[2], \
                float(line.split(',')[0]) - float(first_time), \
                float(line.split(',')[3]), \
                float(self.unwrapped_phase_list[len(self.unwrapped_phase_list) - 1]), 0))

            break

    # 4. Then calculate the slope from delta phase and time
    for index_tag_id in range(len(self.tag_name)):

      if(len(self.tag_list[index_tag_id]) <= 1):
        # if tag_list has the length of 1 it means it has only one object so we can not calculate the slope
        continue
      else:

        # We calculate delta phase by substructing the first and the
        # last unwrapped phases
        dtheta = self.tag_list[index_tag_id][len(self.tag_list[index_tag_id]) - 1].get_unwrapped_phase()\
            - self.tag_list[index_tag_id][0].get_unwrapped_phase()

        # We calculate delta time by substructing the first and the last
        # timed measurements
        dtime = self.tag_list[index_tag_id][len(self.tag_list[index_tag_id]) - 1].get_time()\
            - self.tag_list[index_tag_id][0].get_time()

        # We calculate the slope by dividing delta phase with detla time
        slope = dtheta / dtime

        # We save the slope of the tag in the last measuremnt of that tag
        self.tag_list[index_tag_id][len(self.tag_list[index_tag_id]) - 1].set_slope(slope)

    ###################################################################
    # At this point we have read all txt lines and calculated the slope
    # for all tags
    ###################################################################

    # 5. Find the minimum slope of all tags
    # The min slope gives us the tag that the visitor is looking to.
    # This way  we can tell them which way to look

    min_slope = float('inf')
    min_tag_id = float('inf')

    for i in range(len(self.tag_list)):
      length_list = len(self.tag_list[i])
      if(length_list <= 1):
        continue

      # Find the min slope and the tag id index
      if(self.tag_list[i][len(self.tag_list[i]) - 1].get_slope() < min_slope):
        min_slope = self.tag_list[i][len(self.tag_list[i]) - 1].get_slope()
        min_tag_id = self.tag_list[i][len(self.tag_list[i]) - 1].get_tag_id()

    cl_t2 = time.time()
    exec_time = cl_t2-cl_t1

    # Store results in `self.node_decisions` file
    self.store_results(min_tag_id, min_slope, exec_time)

    # 6. Output the result to the console
    self.direct(min_tag_id, min_slope)

    # clear the lists so they are ready for the next run
    self.clear_structures()

    rospy.loginfo('[iv_rfid_compass] Processing executed in %f ms', 1000*exec_time)


  #-----------------------------------------------------------------------------
  # the function that decides what to rospy.loginfo
  def direct(self, moving_towards_tag_id, min_slope):

    if (moving_towards_tag_id == float('inf') or min_slope == float('inf')):
      rospy.logwarn("[iv_rfid_compass] inf detected; nothing to do" )

    elif(min_slope > self.minimum_slope):
      rospy.logwarn("[iv_rfid_compass] The visitor is standing still." )

    elif(moving_towards_tag_id == self.tag_to_seek):
      # If the visitor is looking at the correct tag
      rospy.logwarn("[iv_rfid_compass] The visitor is moving towards tag number %d", moving_towards_tag_id)
      rospy.logwarn("[iv_rfid_compass] Please keep walking forward!")

    elif(moving_towards_tag_id == self.tag_to_seek + 1):
      # If the visitor is looking at the tag next to the correct one
      # from the right side
      rospy.logwarn("[iv_rfid_compass] The visitor is moving towards tag number %d", moving_towards_tag_id)
      rospy.logwarn("[iv_rfid_compass] Please turn slightly left!")

    elif(moving_towards_tag_id == self.tag_to_seek - 1):
      # If the visitor is looking at the tag  next to the correct one
      # from the left side
      rospy.logwarn("[iv_rfid_compass] The visitor is moving towards tag number %d", moving_towards_tag_id)
      rospy.logwarn("[iv_rfid_compass] Please turn slightly right!")

    elif(moving_towards_tag_id >= self.tag_to_seek + 2 and moving_towards_tag_id <= self.tag_to_seek + 4):
      # If the visitor is looking at the correct tag from the right side,
      # from 2 tags away until 4 tags away
      rospy.logwarn("[iv_rfid_compass] The visitor is moving towards tag number %d", moving_towards_tag_id)
      rospy.logwarn("[iv_rfid_compass] Please turn left!")

    elif(moving_towards_tag_id < self.tag_to_seek):
      # If the visitor is looking at the correct tag from the left side,
      # from 2 tags away until 4 tags away
      rospy.logwarn("[iv_rfid_compass] The visitor is moving towards tag number %d", moving_towards_tag_id)
      rospy.logwarn("[iv_rfid_compass] Please turn right!")

    elif(moving_towards_tag_id > self.tag_to_seek):
      # If the visitor is looking at the tag next to the correct one from
      # the right side
      rospy.logwarn("[iv_rfid_compass] The visitor is moving towards tag number %d", moving_towards_tag_id)
      rospy.logwarn("[iv_rfid_compass] Please make a hard left!")

    elif(moving_towards_tag_id < self.tag_to_seek):
      # If the visitor is looking at the tag  next to the correct one
      # from the left side
      rospy.logwarn("[iv_rfid_compass] The visitor is moving towards tag number %d", moving_towards_tag_id)
      rospy.logwarn("[iv_rfid_compass] Please make a hard right!")

    else:
      rospy.logwarn("[iv_rfid_compass] Cannot see any tag.")
      rospy.logwarn("[iv_rfid_compass] Please rotate yourself before you wreck yourself!")


  #-----------------------------------------------------------------------------
  def process_latest_measurements(self):

    # Copy measurements file to working copy
    shutil.copyfile(self.rfid_measurements_file, self.rfid_measurements_file_working_copy)

    # Append measurements file to readonly file (the readonly file is
    # supposed to be equivalent to the reader's measurements if otherwise
    # not cleared)
    with open(self.rfid_measurements_file_working_copy, 'r') as fr:
      with open(self.rfid_measurements_file_readonly, 'a') as fa:
        for line in fr.readlines():
          fa.write('%s' % line)

    # Erase reader's measurements file contents
    open(self.rfid_measurements_file, 'w').close()

    # Store the timestamp of the new measurement
    with open(self.rfid_measurements_file_working_copy, 'r') as fr:
      self.first_block_measurement_timestamp = fr.readline().split(',')[0]


  #-----------------------------------------------------------------------------
  def store_results(self, min_tag_id, min_slope, exec_time):

    with open(self.node_decisions, 'a') as fa:
      fa.write("%s, %s, %s, %s, %s\n" % (\
          str(rospy.Time.now().to_sec()), \
          str(exec_time), \
          str(min_tag_id), \
          str(min_slope), \
          str(self.first_block_measurement_timestamp)))



################################################################################
# Call me
################################################################################

if __name__ == '__main__':

  rospy.init_node('iv_rfid_compass_node')

  try:
    iv()
    rospy.loginfo('[iv_rfid_compass] Starting to spin')
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo('[iv_rfid_compass] EXCEPTION THROWN')
    pass
