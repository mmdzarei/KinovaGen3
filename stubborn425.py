#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import datetime
#sys.path.append("/home/robot/catkin_workspace/src/aruco_ros/aruco_msgs") # should not be hardcoded

from aruco_msgs.msg import MarkerArray

from tf.transformations import * #euler_from_quaternion, quaternion_from_euler, quaternion_matrix
import math
import numpy

from aruco import *
from is3lkgen3 import *

def dist(p1,p2):
    dist = numpy.linalg.norm(p1-p2)
    return dist

def run():
    global location_x,location_y,location_z
    robot = CartesianActionsWithNotifications()
    # For testing purposes
    success = robot.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
    except:
        pass

    if success:

        #*******************************************************************************
        # Make sure to clear the robot's faults else it won't move if it's already in fault
        success &= robot.example_clear_faults()
        #*******************************************************************************

        #*******************************************************************************
        # Start the example from the Home position
        #success &= robot.example_home_the_robot()
        #*******************************************************************************

        #*******************************************************************************
        # Set the reference frame to "Mixed"
        success &= robot.example_set_cartesian_reference_frame()

        #*******************************************************************************
        # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
        success &= robot.example_subscribe_to_a_robot_notification()

        #*******************************************************************************
        # Prepare and send pose 1

        state=0 #idle
        ar = Aruco()
        faults=0
        while not rospy.is_shutdown():
            rospy.sleep(0.03)
            #   00000

            current_time = datetime.datetime.now()
            print("\rstate=" +str(state) + "  " + str(current_time),end="")
            if state==0:
                try:                                      #Goto my Home
                    robot.goto(0.5,0.0,0.40)
                    robot.gripper_set(0)
                    state=1
                except Exception as e:
                    rospy.logwarn(f"Could not reach home {e}")

                    #   11111
            elif state==1: # wait to reach home
                val = robot.check_for_action_end_or_abort()
                if val == 1:
                    state = 2
                    ar.num_markers = 0 # to make sure that we'll not use the last pose

            elif state == 2:
                if ar.num_markers > 0:
                    cubepose,cubeid=ar.get_marker_by_ord(0)
                    roll,pitch,yaw=euler_from_quaternion([cubepose.orientation.x,
                                                          cubepose.orientation.y,
                                                          cubepose.orientation.z,
                                                          cubepose.orientation.w])
                    matr = quaternion_matrix([cubepose.orientation.x,
                                              cubepose.orientation.y,
                                              cubepose.orientation.z,
                                              cubepose.orientation.w])
                    matt = translation_matrix([cubepose.position.x,
                                               cubepose.position.y,
                                               cubepose.position.z])
                    mattransf = numpy.dot(matt,matr)
                    point = numpy.dot(mattransf,[0.0, 0.0, 0.15, 1.0])

                    roll=math.degrees(roll)
                    pitch=math.degrees(pitch)
                    yaw=math.degrees(yaw)
                    print("goto marker")
                    #print("Final roll=%f, pitch=%f, yaw=%f"%(roll,pitch,yaw))
                    robot.goto(point[0], point[1], point[2], roll+180, pitch, yaw)

                    ar.num_markers = 0
                    state = 3


            elif state == 3:

                val = robot.check_for_action_end_or_abort()
                if val == 1:
                    state = 4
                    ar.clear_markers()


            elif state == 4: # recompute at close distance
                if ar.num_markers > 0:
                    cubepose=ar.get_marker_by_id(cubeid)
                    ar.clear_markers()
                    if (cubepose == None):
                        faults = faults +1
                        continue
                    else:
                        faults = 0

                        if faults >10:
                            state = 0
                        else:
                            roll,pitch,yaw=euler_from_quaternion([cubepose.orientation.x,
                                                                  cubepose.orientation.y,
                                                                  cubepose.orientation.z,
                                                                  cubepose.orientation.w])
                            matr = quaternion_matrix([cubepose.orientation.x,
                                                      cubepose.orientation.y,
                                                      cubepose.orientation.z,
                                                      cubepose.orientation.w])
                            matt = translation_matrix([cubepose.position.x,
                                                       cubepose.position.y,
                                                       cubepose.position.z])
                            mattransf = numpy.dot(matt,matr)
                            point2 = numpy.dot(mattransf,[0.0, 0.00, 0.0, 1.0])
                            roll=math.degrees(roll)
                            pitch=math.degrees(pitch)
                            yaw=math.degrees(yaw)
                            print("final approach")
                            robot.goto(point2[0], point2[1], point2[2], roll+180, pitch, yaw)
                            #print("Final roll=%f, pitch=%f, yaw=%f"%(roll,pitch,yaw))
                            state = 5

            elif state == 5:
                val = robot.check_for_action_end_or_abort()
                if val == 1:
                    state = 6


            elif state ==6:
                if robot.gripper_set(0.44):
                    rospy.loginfo("Cube has been grabbed")
                else:
                    rospy.logwarn("couldn't grab the Cube")
                state = 7

            elif state == 7:
                robot.goto(0.2,-0.5,0.35)
                state=8

            elif state==8: # wait to reach home
                val = robot.check_for_action_end_or_abort()
                if val == 1:
                    state = 9

            elif state==9:
                if robot.gripper_set(0.0):
                    rospy.loginfo("Cube has been dropped")
                    state=0
                else:
                    rospy.logwarn("couldn't drop the Cube")

    # For testing purposes
    rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")

if __name__ == "__main__":

    try:
        run()
    except rospy.ROSInterruptException:
        pass
