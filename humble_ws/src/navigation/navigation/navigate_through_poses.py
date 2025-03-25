"""
This file was adapted from set_goal.py script from the ROS2 navigation Isaac Sim Tutorial
Original file used navigate_to_pose while this one uses navigate_through_poses
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses
import sys
from geometry_msgs.msg import PoseStamped
import time


class SetNavigationGoal(Node):
    def __init__(self):
        super().__init__("set_navigation_goal")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("iteration_count", 1),
                ("action_server_name", "navigate_through_poses"),
                ("obstacle_search_distance_in_meters", 0.2),
                ("frame_id", "map"),
                ("goal_text_file_path", rclpy.Parameter.Type.STRING),
                ("initial_pose", rclpy.Parameter.Type.DOUBLE_ARRAY),
            ],
        )

        self.__goal_generator = self.__create_goal_generator()
        action_server_name = self.get_parameter("action_server_name").value
        self._action_client = ActionClient(self, NavigateThroughPoses, action_server_name)

        self.MAX_ITERATION_COUNT = self.get_parameter("iteration_count").value
        assert self.MAX_ITERATION_COUNT > 0
        self.curr_iteration_count = 1

        self.__initial_goal_publisher = self.create_publisher(PoseStamped, "/initialpose", 1)

        self.__initial_pose = self.get_parameter("initial_pose").value
        self.__is_initial_pose_sent = True if self.__initial_pose is None else False

    def __send_initial_pose(self):
        """
        Publishes the initial pose.
        This function is only called once that too before sending any goal pose
        to the mission server.
        """
        goal = PoseStamped()
        goal.header.frame_id = self.get_parameter("frame_id").value
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.__initial_pose[0]
        goal.pose.position.y = self.__initial_pose[1]
        goal.pose.position.z = self.__initial_pose[2]
        goal.pose.orientation.x = self.__initial_pose[3]
        goal.pose.orientation.y = self.__initial_pose[4]
        goal.pose.orientation.z = self.__initial_pose[5]
        goal.pose.orientation.w = self.__initial_pose[6]
        self.__initial_goal_publisher.publish(goal)

    def send_goal(self):
        """
        Sends the goal to the action server.
        """

        if not self.__is_initial_pose_sent:
            self.get_logger().info("Sending initial pose")
            self.__send_initial_pose()
            self.__is_initial_pose_sent = True

            # Assumption is that initial pose is set after publishing first time in this duration.
            # Can be changed to more sophisticated way. e.g. /particlecloud topic has no msg until
            # the initial pose is set.
            time.sleep(10)
        self.get_logger().info("Sending first goal")

        self._action_client.wait_for_server()
        goal_msg = self.__get_goal()

        if goal_msg is None:
            rclpy.shutdown()
            sys.exit(1)

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.__feedback_callback
        )
        self._send_goal_future.add_done_callback(self.__goal_response_callback)

    def __goal_response_callback(self, future):
        """
        Callback function to check the response(goal accpted/rejected) from the server.\n
        If the Goal is rejected it stops the execution for now.(We can change to resample the pose if rejected.)
        """

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.__get_result_callback)

    def __get_goal(self):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []

        for _ in range(4):  
            pose = self.__goal_generator.generate_goal()
            if pose is None:
                break
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.get_parameter("frame_id").value
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = pose[0]
            pose_stamped.pose.position.y = pose[1]
            pose_stamped.pose.orientation.x = pose[2]
            pose_stamped.pose.orientation.y = pose[3]
            pose_stamped.pose.orientation.z = pose[4]
            pose_stamped.pose.orientation.w = pose[5]
            goal_msg.poses.append(pose_stamped)

        if not goal_msg.poses:
            self.get_logger().error("No valid waypoints found!")
            return None

        return goal_msg

    def __get_result_callback(self, future):
        """
        Callback to check result.\n
        It calls the send_goal() function in case current goal sent count < required goals count.     
        """
        # Nav2 is sending empty message for success as well as for failure.
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result.result))

        if self.curr_iteration_count < self.MAX_ITERATION_COUNT:
            self.curr_iteration_count += 1
            self.send_goal()
        else:
            rclpy.shutdown()

    def __feedback_callback(self, feedback_msg):
        """
        This is feeback callback. We can compare/compute/log while the robot is on its way to goal.
        """
        # self.get_logger().info('FEEDBACK: {}\n'.format(feedback_msg))
        pass

    def __create_goal_generator(self):
        """
        Creates the GoalGenerator object based on the specified ros param value.
        """

        if self.get_parameter("goal_text_file_path").value is None:
            self.get_logger().info("Goal text file path is not given. Returning..")
            sys.exit(1)

        file_path = self.get_parameter("goal_text_file_path").value
        goal_generator = ReadWpt(file_path)

        return goal_generator

class ReadWpt():
    def __init__(self, file_path):
        self.__file_path = file_path
        self.__generator = self.__get_goal()

    def generate_goal(self, max_num_of_trials=1000):
        try:
            return next(self.__generator)
        except StopIteration:
            return

    def __get_goal(self):
        for row in open(self.__file_path, "r"):
            yield list(map(float, row.strip().split(" ")))


def main():
    rclpy.init()
    set_goal = SetNavigationGoal()
    result = set_goal.send_goal()
    rclpy.spin(set_goal)


if __name__ == "__main__":
    main()
