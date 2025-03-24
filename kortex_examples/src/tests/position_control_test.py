#!/usr/bin/env python3
import rospy
import time
import actionlib
from kortex_driver.srv import *
from kortex_driver.msg import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, GripperCommandAction, GripperCommandGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class ExampleFullArmMovement:
    def __init__(self):
        rospy.init_node('example_full_arm_movement')
        self.sim = rospy.get_param('~sim', True)
        self.HOME_ACTION_IDENTIFIER = 2
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
        rospy.loginfo(f"Using robot_name {self.robot_name}, robot has {self.degrees_of_freedom} degrees of freedom")
        self.prefix = rospy.get_param('~prefix', "kinova_")

        self.last_action_notif_type = None

        rospy.wait_for_service(f'/{self.robot_name}/base/clear_faults')
        rospy.wait_for_service(f'/{self.robot_name}/base/read_action')
        rospy.wait_for_service(f'/{self.robot_name}/base/execute_action')
        rospy.wait_for_service(f'/{self.robot_name}/base/activate_publishing_of_action_topic')

        self.clear_faults = rospy.ServiceProxy(f'/{self.robot_name}/base/clear_faults', Base_ClearFaults)
        self.read_action = rospy.ServiceProxy(f'/{self.robot_name}/base/read_action', ReadAction)
        self.execute_action = rospy.ServiceProxy(f'/{self.robot_name}/base/execute_action', ExecuteAction)
        self.activate_notifications = rospy.ServiceProxy(f'/{self.robot_name}/base/activate_publishing_of_action_topic', OnNotificationActionTopic)

        rospy.Subscriber(f"/{self.robot_name}/action_topic", ActionNotification, self.cb_action_topic)

        if self.sim:
            self.client = actionlib.SimpleActionClient(
                f'/{self.robot_name}/{self.prefix}gen3_joint_trajectory_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction
            )
            rospy.loginfo("Waiting for Gazebo joint trajectory action server...")
            self.client.wait_for_server()
            rospy.loginfo("Gazebo joint trajectory action server connected.")

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if self.last_action_notif_type == ActionEvent.ACTION_END:
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif self.last_action_notif_type == ActionEvent.ACTION_ABORT:
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def clear_robot_faults(self):
        try:
            self.clear_faults()
            rospy.loginfo("Cleared faults successfully")
            rospy.sleep(2.5)
            return True
        except rospy.ServiceException:
            rospy.logerr("Failed to clear faults")
            return False

    def subscribe_to_notifications(self):
        req = OnNotificationActionTopicRequest()
        try:
            self.activate_notifications(req)
            rospy.loginfo("Activated action notifications")
            rospy.sleep(1.0)
            return True
        except rospy.ServiceException:
            rospy.logerr("Failed to activate notifications")
            return False

    def home_robot(self):
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
            execute_req = ExecuteActionRequest()
            execute_req.input = res.output
            rospy.loginfo("Sending robot home...")
            self.execute_action(execute_req)
            return self.wait_for_action_end_or_abort()
        except rospy.ServiceException:
            rospy.logerr("Failed to home robot")
            return False

    def send_joint_positions(self, positions, duration=5.0):
        if self.sim:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = [f'{self.prefix}joint_{i+1}' for i in range(self.degrees_of_freedom)]
            point = JointTrajectoryPoint(positions=positions, time_from_start=rospy.Duration(duration))
            goal.trajectory.points.append(point)

            rospy.loginfo(f"Sending simulated robot to positions: {positions}")
            self.client.send_goal(goal)
            self.client.wait_for_result()
            rospy.loginfo("Simulated robot reached target positions.")
        else:
            req = ExecuteActionRequest()
            trajectory = WaypointList()
            waypoint = Waypoint()
            angular_waypoint = AngularWaypoint(angles=positions, duration=duration)
            waypoint.oneof_type_of_waypoint.angular_waypoint.append(angular_waypoint)
            trajectory.waypoints.append(waypoint)

            try:
                req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
                rospy.loginfo(f"Sending real robot to positions: {positions}")
                self.execute_action(req)
                return self.wait_for_action_end_or_abort()
            except rospy.ServiceException:
                rospy.logerr("Failed to send joint angles to real robot")
                return False

    def control_gripper(self, open=True):
        if self.sim:
            gripper_client = actionlib.SimpleActionClient(
                f"/{self.robot_name}/{self.prefix}robotiq_2f_85_gripper_controller/gripper_cmd",
                GripperCommandAction
            )
            rospy.loginfo("Waiting for simulated gripper action server...")
            gripper_client.wait_for_server()
            rospy.loginfo("Simulated gripper action server connected.")

            goal = GripperCommandGoal()
            goal.command.position = 0.0 if open else 0.8  # 0.0=open, ~0.8=closed
            goal.command.max_effort = 50.0
            gripper_client.send_goal(goal)
            gripper_client.wait_for_result()
            rospy.loginfo("Simulated gripper action completed.")
        else:
            try:
                rospy.wait_for_service(f"/{self.robot_name}/base/send_gripper_command", timeout=2)
                send_gripper_cmd = rospy.ServiceProxy(f"/{self.robot_name}/base/send_gripper_command", SendGripperCommand)
                gripper_cmd = GripperCommand()
                finger = Finger()
                finger.finger_identifier = 0
                finger.value = 0.0 if open else 1.0
                gripper_cmd.gripper.finger.append(finger)
                req = SendGripperCommandRequest(input=gripper_cmd)
                send_gripper_cmd(req)
                rospy.loginfo("Gripper command sent to real robot.")
                rospy.sleep(2)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            except rospy.ROSException:
                rospy.logerr("Gripper service not available.")

    def main(self):
        target_positions = [0.0] * self.degrees_of_freedom
        success = self.clear_robot_faults() and self.subscribe_to_notifications() and self.home_robot()
        if success:
            # self.control_gripper(open=True)   # Open gripper
            self.send_joint_positions(target_positions)
            # self.control_gripper(open=False)  # Close gripper
        else:
            rospy.logerr("Encountered an error during initialization.")

if __name__ == "__main__":
    ExampleFullArmMovement().main()
