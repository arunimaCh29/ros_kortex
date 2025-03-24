#!/usr/bin/env python3

import sys
import rospy
import time
import csv
from pathlib import Path
from kortex_driver.srv import *
from kortex_driver.msg import *

class ExampleCartesianActionsWithNotifications:
    def __init__(self):
        try:
            rospy.init_node('example_cartesian_poses_with_notifications_python')

            self.HOME_ACTION_IDENTIFIER = 2
            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            rospy.loginfo("Using robot_name " + self.robot_name)

            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            self.clear_faults = rospy.ServiceProxy('/' + self.robot_name + '/base/clear_faults', Base_ClearFaults)
            rospy.wait_for_service('/' + self.robot_name + '/base/clear_faults')

            self.read_action = rospy.ServiceProxy('/' + self.robot_name + '/base/read_action', ReadAction)
            rospy.wait_for_service('/' + self.robot_name + '/base/read_action')

            self.execute_action = rospy.ServiceProxy('/' + self.robot_name + '/base/execute_action', ExecuteAction)
            rospy.wait_for_service('/' + self.robot_name + '/base/execute_action')

            self.set_cartesian_reference_frame = rospy.ServiceProxy('/' + self.robot_name + '/control_config/set_cartesian_reference_frame', SetCartesianReferenceFrame)
            rospy.wait_for_service('/' + self.robot_name + '/control_config/set_cartesian_reference_frame')

            self.activate_publishing_of_action_notification = rospy.ServiceProxy('/' + self.robot_name + '/base/activate_publishing_of_action_topic', OnNotificationActionTopic)
            rospy.wait_for_service('/' + self.robot_name + '/base/activate_publishing_of_action_topic')

        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if self.last_action_notif_type == ActionEvent.ACTION_END:
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif self.last_action_notif_type == ActionEvent.ACTION_ABORT:
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        rospy.loginfo("Cleared the faults successfully")
        rospy.sleep(2.5)
        return True

    def example_home_the_robot(self):
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        req = ExecuteActionRequest()
        req.input = res.output
        rospy.loginfo("Sending the robot home...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteAction")
            return False
        return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED
        try:
            self.set_cartesian_reference_frame(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        rospy.loginfo("Set the cartesian reference frame successfully")
        rospy.sleep(0.25)
        return True

    def example_subscribe_to_a_robot_notification(self):
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        rospy.loginfo("Successfully activated the Action Notifications!")
        rospy.sleep(1.0)
        return True

    def send_poses_from_csv(self, csv_path):
        try:
            with open(csv_path, newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for i, row in enumerate(reader, start=1):
                    my_cartesian_speed = CartesianSpeed()
                    my_cartesian_speed.translation = 0.1
                    my_cartesian_speed.orientation = 15

                    my_constrained_pose = ConstrainedPose()
                    my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

                    my_constrained_pose.target_pose.x = float(row['x'])
                    my_constrained_pose.target_pose.y = float(row['y'])
                    my_constrained_pose.target_pose.z = float(row['z'])
                    my_constrained_pose.target_pose.theta_x = float(row['theta_x'])
                    my_constrained_pose.target_pose.theta_y = float(row['theta_y'])
                    my_constrained_pose.target_pose.theta_z = float(row['theta_z'])

                    req = ExecuteActionRequest()
                    req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
                    req.input.name = f"pose_{i}"
                    req.input.handle.action_type = ActionType.REACH_POSE
                    req.input.handle.identifier = 1000 + i

                    rospy.loginfo(f"Sending pose {i} from CSV...")
                    self.last_action_notif_type = None
                    try:
                        self.execute_action(req)
                    except rospy.ServiceException:
                        rospy.logerr(f"Failed to send pose {i}")
                        return False
                    rospy.loginfo(f"Waiting for pose {i} to finish...")
                    self.wait_for_action_end_or_abort()

            return True
        except Exception as e:
            rospy.logerr(f"Error reading poses from CSV: {e}")
            return False

    def main(self):
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass

        if success:
            success &= self.example_clear_faults()
            success &= self.example_home_the_robot()
            success &= self.example_set_cartesian_reference_frame()
            success &= self.example_subscribe_to_a_robot_notification()

            # Replace with your actual path to the CSV if different
            csv_file_path = Path("/home/gokul/Desktop/workspace/kinova_ws/src/bags/task_1_bag_1_data/end_effector_poses.csv")
            success &= self.send_poses_from_csv(csv_file_path)

            success &= self.all_notifs_succeeded

        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)
        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = ExampleCartesianActionsWithNotifications()
    ex.main()
