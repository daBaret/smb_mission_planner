from copy import deepcopy
import rospy
from geometry_msgs.msg import PoseStamped

from smb_mission_planner.srv import DetectObject, DetectObjectRequest
from smb_mission_planner.manipulation_states import RosControlPoseReaching
from smb_mission_planner.detection_states import ObjectDetection


class ValveDetectionState(ObjectDetection):
    """
    Detects the valve
    """

    def __init__(self, max_num_failure, ns):
        ObjectDetection.__init__(self, max_num_failure=max_num_failure, ns=ns)

        # TODO(giuseppe) this should be returned by detection as well
        self.valve_radius = self.get_scoped_param("valve_radius")

        self.detection_service_name = self.get_scoped_param("detection_service_name")
        self.detection_service = rospy.ServiceProxy(self.detection_service_name, DetectObject)
        rospy.loginfo("Calling detection service with name: {}".format(self.detection_service_name))

    def execute(self, ud):
        try:
            self.detection_service.wait_for_service(timeout=10.0)
            req = DetectObjectRequest()
            res = self.detection_service.call(req)
            if not res.success:
                rospy.logerr("Valve detection failed")
                self.current_failures += 1
                if self.current_failures < self.max_num_failures:
                    rospy.logwarn("You can retry detection")
                    return 'Retry'
                else:
                    rospy.logerr("Maximum number of detection failures achieved")
                    return 'Aborted'

            self.set_context_data("valve_pose", res.object_pose)
            self.set_context_data("valve_radius", self.valve_radius)
            return 'Completed'

        except rospy.ROSException as exc:
            rospy.logerr(exc)
            return False


class PreGraspState(RosControlPoseReaching):
    """
    Switch and send target poses to the controller manager
    """
    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns)
        pose_topic_name = self.get_scoped_param("pose_topic_name")
        self.pose_goal_publisher = rospy.Publisher(pose_topic_name, PoseStamped, queue_size=1)
        self.vertical_offset = self.get_scoped_param("pre_grasp_vertical_offset")

    def execute(self, ud):
        valve_pose = self.get_context_data("valve_pose")
        valve_radius = self.get_context_data("valve_radius")
        if not valve_pose or not valve_radius:
            return 'Aborted'

        goal = deepcopy(valve_pose)
        goal.pose.position.x += valve_radius
        goal.pose.position.z += self.vertical_offset
        self.pose_goal_publisher.publish(goal)
        rospy.loginfo("Waiting {}s for pose to be reached".format(10.0))
        rospy.sleep(10.0)
        return 'Completed'


class GraspState(RosControlPoseReaching):
    """
    Switch and send the target grasp pose for the end effector
    """
    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns)

    def execute(self, ud):
        valve_pose = self.get_context_data("valve_pose")
        valve_radius = self.get_context_data("valve_radius")
        if not valve_pose or not valve_radius:
            return 'Aborted'

        # TODO(giuseppe)
        return 'Completed'


class ManipulateValve(RosControlPoseReaching):
    """
    Switch and send target poses to the controller manager
    """
    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns)

    def execute(self, ud):
        valve_pose = self.get_context_data("valve_pose")
        valve_radius = self.get_context_data("valve_radius")
        if not valve_pose or not valve_radius:
            return 'Aborted'

        # TODO(giuseppe)
        return 'Completed'
