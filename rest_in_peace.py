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
sys.path.append("/home/robot/catkin_workspace/src/aruco_ros/aruco_msgs")

from aruco_msgs.msg import MarkerArray
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

            self.action_topic_sub0 = rospy.Subscriber("/aruco_marker_publisher/markers_list", UInt32MultiArray, self.aruco_action)
            self.action_topic_sub1 = rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.get_single_aruco_pos)
            

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

        except:
            self.is_init_success = False
        else:
            self.is_init_success = True
            
    def aruco_action(self,data):
        #rospy.loginfo("IDs: %s"%str(data.data))
        for id in data.data:
            if id==64:
                pass
                #rospy.loginfo("IDs: %s"%str(id))
                #print("ID:\t",id)
    def get_single_aruco_pos(self,data):
        self.location_x=None
        self.location_y=None
        self.location_z=None

        if len(data.markers)>0:
            for marker in data.markers:
                self.marker_present=False
                if marker.id==815:
                    self.marker_present=True
                    #rospy.loginfo("Id:\t %s"%str((marker.id)))  
                    self.location_x= (marker.pose.pose.position.x)#+0.003
                    self.location_y= (marker.pose.pose.position.y)
                    self.location_z= (marker.pose.pose.position.z)
                    '''
                    rospy.loginfo("X:\t %s"%str((self.location_x)))   
                    rospy.loginfo("Y:\t %s"%str((self.location_y)))   
                    rospy.loginfo("Z:\t %s"%str((self.location_z)))   
                    '''
                    aruco_pos=f"aruco_pos:\t{(self.location_x)}\t, {(self.location_y)}\t, {(self.location_z)}"
                    #rospy.loginfo(aruco_pos)
                    feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)  
                    self.tool_pose_x=feedback.base.tool_pose_x
                    self.tool_pose_y=feedback.base.tool_pose_y
                    self.tool_pose_z=feedback.base.tool_pose_z
                    tool_pos=f"tool_pos:\t{(self.tool_pose_x)}\t, {(self.tool_pose_y)}\t, {(self.tool_pose_z)}"
                    #rospy.loginfo(tool_pos)
        #pass
    def goto(self,x,y,z,tx=180.0,ty=0.0,tz=90.0):
        my_cartesian_speed = CartesianSpeed()
        my_cartesian_speed.translation = 0.05 # m/s
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
        req.input.name = "poseH"
        req.input.handle.action_type = ActionType.REACH_POSE
        req.input.handle.identifier = 1001

        rospy.loginfo("Sending pose H...")
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send pose H")
            success = False
        else:
            rospy.loginfo("Waiting for pose H to finish...")

            return self.wait_for_action_end_or_abort()
            #return True
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
            success &= self.example_home_the_robot()
            #*******************************************************************************

            #*******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.example_set_cartesian_reference_frame()

            #*******************************************************************************
            # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
            success &= self.example_subscribe_to_a_robot_notification()

            #*******************************************************************************
            # Prepare and send pose 1
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.05 # m/s
            my_cartesian_speed.orientation = 15  # deg/s
            rate=rospy.Rate(20)
            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

            my_constrained_pose.target_pose.x = 0.66
            my_constrained_pose.target_pose.y = 0
            my_constrained_pose.target_pose.z = 0.04
            my_constrained_pose.target_pose.theta_x = 90
            my_constrained_pose.target_pose.theta_y = 0.0
            my_constrained_pose.target_pose.theta_z = 90.0

            req = ExecuteActionRequest()
            req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
            req.input.name = "rest"
            req.input.handle.action_type = ActionType.REACH_POSE
            req.input.handle.identifier = 1001

            rospy.loginfo("Sending rest pose ...")
            self.last_action_notif_type = None
            try:
                    self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to send rest pose")
                success = False
            else:
                rospy.loginfo("Waiting for rest pose to finish...")
             

            success &= self.all_notifs_succeeded

            success &= self.all_notifs_succeeded

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
