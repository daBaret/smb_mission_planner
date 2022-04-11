#!/usr/bin/env python
import smach
import rospy
import tf
import smach
from geometry_msgs.msg import PoseStamped
import tf2_ros
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalStatus


class MissionPlan():
    def __init__(self, missions_data):
        self.missions_data = missions_data

    def createStateMachine(self):
        state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])
        with state_machine:
            smach.StateMachine.add('Mission 1', DefaultMission(self.missions_data['test_mission']),
                                   transitions={'Completed': 'Success', 'Aborted': 'Failure', 'Next Waypoint': 'Mission 1'})

        return state_machine


class DefaultMission(smach.State):
    def __init__(self, mission_data):
        smach.State.__init__(
            self, outcomes=['Completed', 'Aborted', 'Next Waypoint'])

        # Get parameters
        # Frames
        self.base_frame = rospy.get_param('~frames/base', 'base_link')
        self.reference_frame = rospy.get_param('~frames/reference', 'map')

        # Settings
        self.countdown_s = rospy.get_param('~settings/skip_countdown', 60)
        self.countdown_decrement_s = rospy.get_param(
            '~settings/skip_countdown_decrement', 1)

        # Topics
        self.waypoint_topic = rospy.get_param(
            'topics/waypoint', '/move_base_simple/goal')
        self.status_topic = rospy.get_param(
            'topics/status', '/move_base/status')

        self.mission_data = mission_data
        self.waypoint_idx = 0
        self.goal_status = 0

        # Subscribe to move_base status
        self.status_subscribrt = rospy.Subscriber(
            self.status_topic, GoalStatusArray, self.statusCallback)

        self.waypoint_pose_publisher = rospy.Publisher(
            self.waypoint_topic, PoseStamped, queue_size=1)
        while self.waypoint_pose_publisher.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo_once("Waiting for subscriber to connect to '" +
                               self.waypoint_topic + "'.")
            rospy.sleep(1)

    def execute(self, userdata):
        if(self.waypoint_idx >= len(self.mission_data.keys())):
            rospy.loginfo("No more waypoints left in current mission.")
            self.waypoint_idx = 0
            return 'Completed'

        current_waypoint_name = list(self.mission_data.keys())[
            self.waypoint_idx]
        current_waypoint = self.mission_data[current_waypoint_name]

        self.setWaypoint(
            current_waypoint['x_m'], current_waypoint['y_m'], current_waypoint['yaw_rad'])
        rospy.loginfo("Waypoint set: '" + current_waypoint_name + "'.")

        while self.goal_status != GoalStatus.ACTIVE:
            rospy.sleep(1)
            rospy.loginfo_throttle(1, "Waiting for new waypoint to be processed")

        countdown_s = self.countdown_s
        while countdown_s and not rospy.is_shutdown():
            if(self.reachedWaypointWithTolerance()):
                rospy.loginfo("Waypoint '" + current_waypoint_name +
                              "' reached before countdown ended. Loading next waypoint...")
                self.waypoint_idx += 1
                return 'Next Waypoint'
            else:
                rospy.loginfo(str(
                    countdown_s) + "s left until skipping waypoint '" + current_waypoint_name + "'.")
                rospy.sleep(self.countdown_decrement_s)
            countdown_s -= self.countdown_decrement_s
        rospy.logwarn("Countdown ended without reaching waypoint '" +
                      current_waypoint_name + "'.")
        if(self.waypoint_idx == 0):
            rospy.logwarn(
                "Starting waypoint of mission unreachable. Aborting current mission.")
            self.waypoint_idx = 0.
            return 'Aborted'
        else:
            rospy.logwarn("Skipping waypoint '" + current_waypoint_name + "'.")
            self.waypoint_idx += 1
            return 'Next Waypoint'

    def setWaypoint(self, x_m, y_m, yaw_rad):
        quaternion = tf.transformations.quaternion_from_euler(0., 0., yaw_rad)

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.seq = 0
        pose_stamped_msg.header.stamp.secs = rospy.get_rostime().secs
        pose_stamped_msg.header.stamp.nsecs = rospy.get_rostime().nsecs
        pose_stamped_msg.header.frame_id = self.reference_frame
        pose_stamped_msg.pose.position.x = x_m
        pose_stamped_msg.pose.position.y = y_m
        pose_stamped_msg.pose.position.z = 0.
        pose_stamped_msg.pose.orientation.x = quaternion[0]
        pose_stamped_msg.pose.orientation.y = quaternion[1]
        pose_stamped_msg.pose.orientation.z = quaternion[2]
        pose_stamped_msg.pose.orientation.w = quaternion[3]
        self.waypoint_pose_publisher.publish(pose_stamped_msg)

        self.waypoint_x_m = x_m
        self.waypoint_y_m = y_m
        self.waypoint_yaw_rad = yaw_rad

    def statusCallback(self, status_msgs):
        self.goal_status = status_msgs.status_list[-1].status

    def reachedWaypointWithTolerance(self):
        if self.goal_status == GoalStatus.SUCCEEDED:
            goal_reached = True
        else:
            goal_reached = False
        return goal_reached
