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
import rospy
import time
from kortex_driver.srv import *
from kortex_driver.msg import *
from std_msgs.msg import UInt32MultiArray
#sys.path.append("/home/robot/catkin_workspace/src/aruco_ros/aruco_msgs") # should not be hardcoded

from aruco_msgs.msg import MarkerArray

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class Aruco:
    def __init__(self):
        self.name="Lixo"
        try:
            rospy.loginfo("Init Aruco class")
            #self.action_topic_sub0 = rospy.Subscriber("/aruco_marker_publisher/markers_list", UInt32MultiArray, self.aruco_action)

            # self.action_topic_sub1 = rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.get_single_aruco_pos)

            self.action_topic_sub2 = rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.get_arucos)
            
            self.num_markers = 0
        except:
            rospy.loginfo("Aruco: Failed init")
            self.is_init_success = False
        else:
            self.is_init_success = True
            
            
    def aruco_action(self,data):
        for id in data.data:
            if id==64:
                pass
                # rospy.loginfo("IDs: %s"%str(id))
                #print("ID:\t",id)
                
    def get_arucos(self,data):
        self.num_markers = len(data.markers)
        if self.num_markers > 0:
            #print("Markers.len="+str(len(data.markers)) + " " + str(self.num_markers))
            self.markers=data.markers
           
        else:
            print("no:"+str(data))

          
    def check_aruco(self,marker_no):
        self.marker_position=None
        if len(self.markers)>0:
            for marker in self.markers:
                if marker.id==marker_no:
                    self.marker_position= (marker.pose.pose.position)
                    rospy.loginfo(f"Marker {marker_no} has been found. position= {self.marker_position}")

    def get_marker_by_ord(self, ord):
        if self.num_markers >= ord:
           # print(self.num_markers)
            marker = self.markers[ord]
            #print(marker.pose)
            return marker.pose.pose
        else:
            return None




class ExampleCartesianActionsWithNotifications:
    def __init__(self):
        
        try:
            rospy.init_node('example_cartesian_poses_with_notifications_python')

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")

            rospy.loginfo("Using robot_name " + self.robot_name)

            

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)


            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            self.location_x=None
            self.location_y=None
            self.location_z=None

            feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)  
            self.tool_pose_x=feedback.base.tool_pose_x
            self.tool_pose_y=feedback.base.tool_pose_y
            self.tool_pose_z=feedback.base.tool_pose_z
            self.tool_pose_theta_x=feedback.base.tool_pose_theta_x
            self.tool_pose_theta_y=feedback.base.tool_pose_theta_y
            self.tool_pose_theta_z=feedback.base.tool_pose_theta_z

            self.marker_present=False
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True


        
    def goto(self,x,y,z,tx=180.0,ty=0.0,tz=90.0):
       
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.25 # m/s
            my_cartesian_speed.orientation = 15  # deg/s
            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)
            my_constrained_pose.target_pose.x = x
            my_constrained_pose.target_pose.y = y
            my_constrained_pose.target_pose.z = z
            my_constrained_pose.target_pose.theta_x = tx
            my_constrained_pose.target_pose.theta_y = ty
            my_constrained_pose.target_pose.theta_z = tz

            req = ExecuteActionRequest()
            req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
            req.input.name = "poseHHH"
            req.input.handle.action_type = ActionType.REACH_POSE
            req.input.handle.identifier = 1001

            rospy.loginfo("Sending pose HHH...")
            self.last_action_notif_type = None
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to send pose HHH")
                success = False
            else:
                rospy.loginfo("Waiting for pose HHH to finish...")

                #return self.wait_for_action_end_or_abort()
       

    def goto_aruco(self,x,y,z,tx=180.0,ty=0.0,tz=90.0):
        if (abs(self.tool_pose_x-self.location_x)>0.005 or abs(self.tool_pose_y-self.location_y)>0.005):
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.05 # m/s
            my_cartesian_speed.orientation = 15  # deg/s
            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)
            my_constrained_pose.target_pose.x = self.location_x
            my_constrained_pose.target_pose.y = self.location_y
            my_constrained_pose.target_pose.z = z
            my_constrained_pose.target_pose.theta_x = tx
            my_constrained_pose.target_pose.theta_y = ty
            my_constrained_pose.target_pose.theta_z = tz

            req = ExecuteActionRequest()
            req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
            req.input.name = "poseHHH"
            req.input.handle.action_type = ActionType.REACH_POSE
            req.input.handle.identifier = 1001

            rospy.loginfo("Sending pose HHH...")
            self.last_action_notif_type = None
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to send pose HHH")
                success = False
            else:
                rospy.loginfo("Waiting for pose HHH to finish...")

                return self.wait_for_action_end_or_abort()
        else:
            rospy.loginfo("Position has been reached")
                #return True
    def grab_cube(self,grip):
        return self.example_send_gripper_command(grip)
    def drop_cube(self,grip):
        self.example_send_gripper_command(grip)
    def raise_up(self,height):
        my_cartesian_speed = CartesianSpeed()
        my_cartesian_speed.translation = 0.05 # m/s
        my_cartesian_speed.orientation = 15  # deg/s
        my_constrained_pose = ConstrainedPose()
        my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)
        my_constrained_pose.target_pose.x = self.tool_pose_x
        my_constrained_pose.target_pose.y = self.tool_pose_y
        my_constrained_pose.target_pose.z = height
        my_constrained_pose.target_pose.theta_x = self.tool_pose_theta_x
        my_constrained_pose.target_pose.theta_y = self.tool_pose_theta_y
        my_constrained_pose.target_pose.theta_z = self.tool_pose_theta_z

        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
        req.input.name = "safe height"
        req.input.handle.action_type = ActionType.REACH_POSE
        req.input.handle.identifier = 1002

        rospy.loginfo("Sending safe height ...")
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send safe height")
            success = False
        else:
            rospy.loginfo("Waiting for raise_up to finish...")
            return self.wait_for_action_end_or_abort()
        
    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

    def check_for_action_end_or_abort(self):
        if (self.last_action_notif_type == ActionEvent.ACTION_END):
            rospy.loginfo("Received ACTION_END notification")
            return 1
        elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
            rospy.loginfo("Received ACTION_ABORT notification")
            self.all_notifs_succeeded = False
            return 2
        else:
            return 0

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True

        # Wait a bit
        rospy.sleep(0.25)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)

        return True
    #def example_subscribe_to_aruco(self):

    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    def main(self):
        global location_x,location_y,location_z
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass

        if success:
            
            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            #*******************************************************************************
            
            #*******************************************************************************
            # Start the example from the Home position
            #success &= self.example_home_the_robot()
            #*******************************************************************************

            #*******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.example_set_cartesian_reference_frame()

            #*******************************************************************************
            # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
            success &= self.example_subscribe_to_a_robot_notification()

            #*******************************************************************************
            # Prepare and send pose 1

            state=0 #idle
            ar = Aruco()
            
            while not rospy.is_shutdown():
                    rospy.sleep(0.5)
                    #   00000
                    print("state=" +str(state))
                    if state==0:          
                        try:                                      #Goto my Home 
                            self.goto(0.5,0.0,0.40)
                            state=1
                        except Exception as e:
                            rospy.logwarn(f"Could not reach home {e}")

                    #   11111         
                    elif state==1: # wait to reach home
                        val = self.check_for_action_end_or_abort()
                        if val == 1:
                            state = 2
                            ar.num_markers = 0 # to make sure that we'll not use the last pose

                    elif state == 2:
                        if ar.num_markers > 0:
                            pose=ar.get_marker_by_ord(0)
                            self.roll,self.pitch,self.yaw=euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
                            #self.roll,self.pitch,self.yaw=self.orientation
                            print(self.roll)
                            print(self.pitch)
                            print(self.yaw)
                            self.roll=math.degrees(self.roll)
                            self.pitch=math.degrees(self.pitch)
                            self.yaw=math.degrees(self.yaw)
                            print("goto marker")
                            self.goto(pose.position.x, pose.position.y, pose.position.z+0.0,self.roll+180, self.pitch, self.yaw)
                            # self.goto(pose.position.x, pose.position.y, pose.position.z+0.0,90, 0, 90)
                            ar.num_markers = 0
                            state = 3

                            
                    elif state == 3:
                        val = self.check_for_action_end_or_abort()
                        if val == 1:
                            state = 4
                        

                    elif state ==4:
                        if self.grab_cube(0.53):
                            rospy.loginfo("Cube has been grabbed")
                            state=5
                        else:
                            rospy.logwarn("couldn't grab the Cube")
                        state = 5
                  
                    elif state == 5:
                        state = 6
                        self.goto(0.2,-0.5,0.30)
                        state=7
                        
                    elif state==7: # wait to reach home
                        val = self.check_for_action_end_or_abort()
                        if val == 1:
                            state = 8
                                                        
                    elif state==8:
                        if self.grab_cube(0.0):
                            rospy.loginfo("Cube has been dropped")
                            state=0
                        else:
                            rospy.logwarn("couldn't drop the Cube")

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = ExampleCartesianActionsWithNotifications()
    try:
        ex.main()
    except rospy.ROSInterruptException:
        pass
