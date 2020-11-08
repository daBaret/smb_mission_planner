#!/usr/bin/env python
from smb_mission_planner.base_state_ros import BaseStateRos

"""
Here define all the detection related states
"""


class ObjectDetection(BaseStateRos):
    """
    Basic detection states which will output to retry until the maximum number of allowed failure is reached
    """
    def __init__(self, max_num_failure=1, ns=""):
        BaseStateRos.__init__(self,
                              outcomes=['Completed', 'Aborted', 'Retry'],
                              ns=ns)

        self.current_failures = 0
        self.max_num_failures = max_num_failure

    def execute(self, ud):
        raise NotImplementedError("The execute method needs to be implemented")
