#!/usr/bin/env python

import threading
import rospy
import actionlib
import actionlib_msgs
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray ,PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Float64, Int16
from std_srvs.srv import Empty as Empty_srv
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import math
import rospkg
import csv
import time


#Path for saving and retrieving the pose.csv file
output_file_path = rospkg.RosPack().get_path('follow_waypoints')+"/saved_path/pose.csv"
waypoints = []
waypoints_duration = 0.0
waypoints_iterations = 1

################################################################################
################################################################################
class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'navigation_paused', 'return_home'], input_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listner.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = rospy.get_param('waypoint_distance_tolerance', 0.2)
        self.cancel_goal_publisher = rospy.Publisher('/move_base/cancel', actionlib_msgs.msg.GoalID, queue_size=1)
        self.navigation_paused = False
        self.navigation_halted = False

        # Start thread to listen for pause messages
        def wait_for_path_pause():
            """thread worker function"""
            while not rospy.is_shutdown():
                rospy.wait_for_message('/waypoints_pause_navigation', Empty)
                rospy.loginfo('Received path PAUSE message')
                self.navigation_paused = True

                # Send a cancel msg to move base;
                # This will cause 'self.client.wait_for_result()' to
                # return (it is blocking)
                gid = actionlib_msgs.msg.GoalID()
                self.cancel_goal_publisher.publish(gid)

        pause_thread = threading.Thread(target=wait_for_path_pause)
        pause_thread.start()

        # Start thread to listen for halt messages
        def wait_for_path_halt():
            """thread worker function"""
            while not rospy.is_shutdown():
                rospy.wait_for_message('/waypoints_halt_navigation', Empty)
                rospy.loginfo('RECEIVED PATH HALT MESSAGE')
                self.navigation_halted = True

                # Send a cancel msg to move base;
                # This will cause 'self.client.wait_for_result()' to
                # return (it is blocking)
                gid = actionlib_msgs.msg.GoalID()
                self.cancel_goal_publisher.publish(gid)

        halt_thread = threading.Thread(target=wait_for_path_halt)
        halt_thread.start()



    def execute(self, userdata):
        global waypoints
        global waypoints_duration
        global waypoints_iterations
        global navigation_started_at
        global current_iteration
        global current_iterations_tot
        global waypoint_counter
        global returned_home


        # We must keep track of the waypoint_counter because when returning
        # back to this state from a pause state the waypoint counter gets
        # destroyed. We make waypoint_counter a global variable and at
        # initialisation only it gets initialised to 0. Navigation from a
        # resumed state to the same waypoints_iterations is facilitated by the
        # below statement within the for:
        # if i < waypoint_counter:
        #   continue
        if not "waypoint_counter" in globals():
          waypoint_counter = 0

        # Execute waypoints each in sequence
        #for waypoint in waypoints.poses:
        for i in range(0,len(waypoints.poses)):

            if i < waypoint_counter:
              continue

            waypoint = waypoints.poses[i]

            # Commence navigation

            # If a duration (hours) has been set, and if navigation has exceeded
            # the desired duration, then stop navigation
            if waypoints_duration > 0.0:
              if (rospy.Time.now().secs - navigation_started_at.secs) > waypoints_duration*3600 and returned_home == False:
                return 'return_home'

            # Break if preempted
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break

            # Otherwise publish next waypoint as goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.pose.position.x, waypoint.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal)

            # Will return if robot stuck (goal aborted)
            if not self.distance_tolerance > 0.0:
                self.client.wait_for_result()
            else:
                # Near a certain GOAL point.
                distance = self.distance_tolerance + 1
                while distance > pow(self.distance_tolerance,2) and self.navigation_paused == False and self.navigation_halted == False:
                    now = rospy.Time.now()
                    self.listener.waitForTransform(self.frame_id, self.base_frame_id, now, rospy.Duration(4.0))
                    trans,rot = self.listener.lookupTransform(self.frame_id,self.base_frame_id, now)
                    distance = pow(waypoint.pose.position.x-trans[0],2)+pow(waypoint.pose.position.y-trans[1],2)

                    # This loop: by default will *not* return if robot stuck (goal aborted);
                    # So catch pause and halt signals
                    if self.navigation_paused == True or self.navigation_halted == True:
                      break

            if self.navigation_paused == True:
              self.navigation_paused = False
              return 'navigation_paused'

            if self.navigation_halted == True:
              self.navigation_halted = False
              return 'success'

            rospy.loginfo("Waiting for %f sec..." % self.duration)
            time.sleep(self.duration)

            waypoint_counter = waypoint_counter + 1
            current_iterations_tot = current_iterations_tot + 1
            if current_iterations_tot % len(waypoints.poses) / waypoints_iterations == 0:
              current_iteration = current_iteration + 1

        if returned_home == True:
          return 'success'
        else:
          return 'return_home'


################################################################################
################################################################################
class PauseNavigation(State):
    def __init__(self):
        State.__init__(self, outcomes=['navigation_resumed', 'return_home'], input_keys=['waypoints'])
        self.navigation_resumed = False
        self.return_home = False

        # Start listening for navigation resume messages
        def wait_for_navigation_resumed():
            """thread worker function"""
            while not rospy.is_shutdown():
              rospy.wait_for_message('/waypoints_resume_navigation', Empty)
              rospy.loginfo('Received path RESUME message')
              self.navigation_resumed = True

        nav_resume_thread = threading.Thread(target=wait_for_navigation_resumed)
        nav_resume_thread.start()

        # Start thread to listen for return home messages
        def wait_for_return_home():
            """thread worker function"""
            while not rospy.is_shutdown():
              rospy.wait_for_message('/waypoints_return_home', Empty)
              rospy.loginfo('Received path RETURN HOME message')
              self.return_home = True

        return_home_thread = threading.Thread(target=wait_for_return_home)
        return_home_thread.start()


    # Start listening for resume messages; stay here until you do
    def execute(self, userdata):

        self.navigation_resumed = False
        self.return_home = False

        while not rospy.is_shutdown():
          while self.navigation_resumed == False and self.return_home == False:
            rospy.sleep(2)

          if self.navigation_resumed == True:
            # Wait for five seconds for the person to clear the front of the
            # robot. Then clear her footprint. Then wait another five seconds
            # so that the robot plans with full costmaps (otherwise it may
            # plan through obstacles)
            rospy.loginfo('Waiting for 10 sec before resuming navigation')
            rospy.sleep(5)

            clear_costmaps_srv = '/move_base/clear_costmaps'
            rospy.wait_for_service(clear_costmaps_srv)
            try:
              rospy.loginfo('Clearing costmaps')
              cc = rospy.ServiceProxy(clear_costmaps_srv, Empty_srv)
              cc()
            except rospy.ServiceException as e:
              rospy.loginfo("Service call costmap clearance failed: %s"%e)

            rospy.sleep(5)
            return 'navigation_resumed'

          if self.return_home == True:
            # Wait for five seconds for the person to clear the front of the
            # robot. Then clear her footprint. Then wait another five seconds
            # so that the robot plans with full costmaps (otherwise it may
            # plan through obstacles)
            rospy.loginfo('Waiting for 10 sec before returning home')
            rospy.sleep(5)

            clear_costmaps_srv = '/move_base/clear_costmaps'
            rospy.wait_for_service(clear_costmaps_srv)
            try:
              rospy.loginfo('Clearing costmaps')
              cc = rospy.ServiceProxy(clear_costmaps_srv, Empty_srv)
              cc()
            except rospy.ServiceException as e:
              rospy.loginfo("Service call costmap clearance failed: %s"%e)

            rospy.sleep(5)
            return 'return_home'

################################################################################
################################################################################
def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id','map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses

################################################################################
################################################################################
def convert_Path_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id','map')
    poses.poses = [pose.pose for pose in waypoints.poses]
    return poses

################################################################################
################################################################################
class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])

        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.poseArray_publisher = rospy.Publisher('/waypoints_2', PoseArray, queue_size=1)

        # For self-publishing the required message; added 01/10/2020
        self.empty_msg_pub = rospy.Publisher('/path_ready', Empty, queue_size=1)

        """
        # Start thread to listen for reset messages to clear the waypoint queue
        def wait_for_path_reset():
            # thread worker function
            global waypoints
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/path_reset', Empty)
                rospy.loginfo('Recieved path RESET message')
                self.initialize_path_queue()
                rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                               # for three seconds and wait_for_message() in a
                               # loop will see it again.
        reset_thread = threading.Thread(target=wait_for_path_reset)
        reset_thread.start()
        """

    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # the waypoint queue
        # publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        #self.poseArray_publisher.publish(convert_Path_to_PoseArray(waypoints))

    def execute(self, userdata):
        global waypoints
        global waypoints_duration
        global waypoints_iterations
        global navigation_started_at
        global current_iteration
        global current_iterations_tot
        self.initialize_path_queue()
        self.path_ready = False

        global returned_home
        returned_home = False

        # Start thread to listen for when the path is ready (this function will end then)
        # Also will save the clicked path to pose.csv file
        def wait_for_path_ready():
            """thread worker function"""
            data = rospy.wait_for_message('/path_ready', Empty)
            rospy.loginfo('Received path READY message')
            rospy.loginfo('Waiting for 10 sec before starting navigation to selected waypoints')

            # The home pose should be set before setting the waypoint sequence.
            # Executing the next three lines AT THIS POINT IN THE CODE will
            # result in setting the home pose to the instantaneous pose of the
            # robot at the point where the Start Navigation button was
            # pressed.
            # Alternatively:
            # If these lines are put at the level of the GetPath function,
            # then the robot will return to its origin every time
            # FUTURE TBD
            global home_pose
            home_pose_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
            home_pose = home_pose_msg.pose.pose

            # Wait for five seconds for the person to clear the front of the
            # robot. Then clear her footprint. Then wait another five seconds
            # so that the robot plans with full costmaps (otherwise it may
            # plan through obstacles)
            rospy.sleep(5)

            clear_costmaps_srv = '/move_base/clear_costmaps'
            rospy.wait_for_service(clear_costmaps_srv)
            try:
              rospy.loginfo('Clearing costmaps')
              cc = rospy.ServiceProxy(clear_costmaps_srv, Empty_srv)
              cc()
            except rospy.ServiceException as e:
              rospy.loginfo("Service call costmap clearance failed: %s"%e)

            rospy.sleep(5)

            self.path_ready = True

            with open(output_file_path, 'w') as file:
                for current_pose in waypoints.poses:
                    file.write(str(current_pose.pose.position.x) + ',' + str(current_pose.pose.position.y) + ',' + str(current_pose.pose.position.z) + ',' + str(current_pose.pose.orientation.x) + ',' + str(current_pose.pose.orientation.y) + ',' + str(current_pose.pose.orientation.z) + ',' + str(current_pose.pose.orientation.w)+ '\n')
	        rospy.loginfo('poses written to '+ output_file_path)
        ready_thread = threading.Thread(target=wait_for_path_ready)
        ready_thread.start()


        # Initialise for recurrent use/check
        navigation_started_at = rospy.Time.now()
        current_iteration = 0
        current_iterations_tot = 0

        self.start_journey_bool = False

        """
        # Start thread to listen start_jorney
        # for loading the saved poses from follow_waypoints/saved_path/poses.csv
        def wait_for_start_journey():
            # thread worker function
            data_from_start_journey = rospy.wait_for_message('/start_journey', Empty)
            rospy.loginfo('Recieved path READY start_journey')
            global waypoints
            waypoints = Path()
            with open(output_file_path, 'r') as file:
                reader = csv.reader(file, delimiter = ',')
                for row in reader:
                    print row
                    current_pose = PoseStamped()
                    current_pose.pose.position.x     =    float(row[0])
                    current_pose.pose.position.y     =    float(row[1])
                    current_pose.pose.position.z     =    float(row[2])
                    current_pose.pose.orientation.x = float(row[3])
                    current_pose.pose.orientation.y = float(row[4])
                    current_pose.pose.orientation.z = float(row[5])
                    current_pose.pose.orientation.w = float(row[6])
                    waypoints.poses.append(current_pose)
                    #self.poseArray_publisher.publish(convert_Path_to_PoseArray(waypoints))
            self.start_journey_bool = True


        start_journey_thread = threading.Thread(target=wait_for_start_journey)
        start_journey_thread.start()
        """

        topic = "/waypoints"
        rospy.loginfo("Waiting to receive waypoints via Pose msg on topic %s" % topic)
        #rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")
        #rospy.loginfo("OR")
        #rospy.loginfo("To start following saved waypoints: 'rostopic pub /start_journey std_msgs/Empty -1'")


        # Wait for published waypoints or saved path  loaded
        while (not self.path_ready and not self.start_journey_bool):
            try:
                waypoints = rospy.wait_for_message(topic, Path, timeout=1)
                wp_its_msg = rospy.wait_for_message('/waypoints_iterations', Int16)
                waypoints_iterations = wp_its_msg.data
                wp_dur_msg = rospy.wait_for_message('/waypoints_duration', Float64)
                waypoints_duration = wp_dur_msg.data

                # By default the waypoints_duration is 0. This means that
                # only iterated-waypoints navigation will occur.
                # ** If however the duration has been set, then we will follow
                # waypoints based on duration and waypoints_iterations will be
                # discarded. **
                # If the duration has been set:
                # 1) the waypoints will be iterated and
                # 2) we do not know in advance how many times.
                # We cannot break the logic of waypoint navigation here, so
                # if the duration has been set we give an adequately large
                # number of iterations and we check the current time against the
                # duration
                if waypoints_duration > 0.0:
                  waypoints_iterations = 100000

                # Repeat content of waypoints msg for waypoints_iterations times
                waypoints_rep = Path()
                for i in range(0,waypoints_iterations):
                  for waypoint in waypoints.poses:
                    waypoints_rep.poses.append(waypoint)

                waypoints = waypoints_rep

            except rospy.ROSException as e:
                if 'timeout exceeded' in e.message:
                    continue  # no new waypoint within timeout, looping...
                else:
                    raise e
            rospy.loginfo("Received %s waypoints" % str(len(waypoints.poses) / waypoints_iterations))

            # publish waypoint queue as pose array so that you can see them in rviz, etc.
            #self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
            #self.poseArray_publisher.publish(convert_Path_to_PoseArray(waypoints))

            # Message by li9i, 1/10/2020:
            # Once the waypoints message is received, this package be default
            # awaits for the publication of an empty message to commence
            # waypoint following. But the waypoints are published all at once
            # from the waypoint_navigation_plugin, contrary to the intended
            # logic of the default package. Therefore after receiving the
            # waypoints self-publish the empty message; and following will
            # commence at once
            if (waypoints.poses) > 0:
              empty_msg = Empty()
              self.empty_msg_pub.publish(empty_msg)

        # Path is ready! return success and move on to the next state (FOLLOW_PATH)
        return 'success'

################################################################################
################################################################################
class ReturnHome(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('###############################')
        rospy.loginfo('#### IMMA COMING HOME MAMA ####')
        rospy.loginfo('###############################')

        global waypoints
        global waypoints_iterations
        global waypoints_duration
        global waypoint_counter
        global returned_home
        global home_pose

        return_method = 0

        # Just return to the home pose however way you can
        if return_method == 0:
          waypoints = Path()

          home_pose_stamped_msg = PoseStamped()
          home_pose_stamped_msg.header.stamp = rospy.Time.now()
          home_pose_stamped_msg.pose = home_pose
          waypoints.poses.append(home_pose_stamped_msg)


        # Return home via the waypoints you just passed until you get to your
        # first waypoint, which SHOULD be the closest to the home pose
        if return_method == 1:

          waypoints_size = 0
          if waypoints_duration > 0.0:
            waypoints_size = len(waypoints.poses) / 100000
          else:
            waypoints_size = len(waypoints.poses) / waypoints_iterations

          # This is the index of the waypoint that the robot just passed through
          # (the counter points at non repeated waypoints though)
          current_wp_counter = waypoint_counter % waypoints_size

          wp = Path()
          wp = waypoints

          if current_wp_counter == 0:
            # If the robot is returning home after completing its run
            if waypoint_counter > 0:
              wp.poses = waypoints.poses[0:waypoints_size]
            else:
              wp.poses = []
          else:
            # If the robot is returning home without having completed its run
            wp.poses = wp.poses[0:current_wp_counter]

          wp.poses.reverse()

          # Since the robot is returning, make the  orientation + 180 degrees
          for i in range(0,len(wp.poses)):
            orientation_q = wp.poses[i].pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            roll = 0.0
            pitch = 0.0
            yaw = 0.0
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

            yaw = yaw + 3.1415

            quat = quaternion_from_euler (roll, pitch,yaw)
            wp.poses[i].pose.orientation.x = quat[0]
            wp.poses[i].pose.orientation.y = quat[1]
            wp.poses[i].pose.orientation.z = quat[2]
            wp.poses[i].pose.orientation.w = quat[3]

          home_pose_stamped_msg = PoseStamped()
          home_pose_stamped_msg.header.stamp = rospy.Time.now()
          home_pose_stamped_msg.pose = home_pose
          wp.poses.append(home_pose_stamped_msg)

          waypoints = wp


        waypoint_counter = 0
        returned_home = True

        return 'success'

################################################################################
################################################################################
class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')

        # Reset counters
        global current_iteration
        global current_iterations_tot
        global waypoint_counter
        global waypoints_duration
        global waypoints_iterations
        global returned_home

        current_iteration = 0
        current_iterations_tot = 0
        waypoint_counter = 0
        waypoints_duration = 0.0
        waypoints_iterations = 0
        returned_home = False

        return 'success'

################################################################################
################################################################################
def main():
    rospy.init_node('follow_waypoints')

    sm = StateMachine(outcomes=['success', 'navigation_paused', 'navigation_resumed', 'return_home'])

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                           transitions={'success':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints'})

        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'success':'PATH_COMPLETE',
                                        'navigation_paused':'NAVIGATION_PAUSED',
                                        'return_home':'RETURN_HOME'},
                           remapping={'waypoints':'waypoints'})

        StateMachine.add('NAVIGATION_PAUSED', PauseNavigation(),
                           transitions={'navigation_resumed':'FOLLOW_PATH',
                                        'return_home':'RETURN_HOME'},
                           remapping={'waypoints':'waypoints'})

        StateMachine.add('RETURN_HOME', ReturnHome(),
                           transitions={'success':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints'})

        StateMachine.add('PATH_COMPLETE', PathComplete(),
                           transitions={'success':'GET_PATH'})

    outcome = sm.execute()
