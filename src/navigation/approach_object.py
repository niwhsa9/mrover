import tf2_ros

from util.ros_utils import get_rosparam
from util.state_lib.state import State
from typing import Optional

from navigation import state
from navigation.approach_target_base import ApproachTargetBaseState


class ApproachObjectState(ApproachTargetBaseState):
    """
    State for when we see an object (mallet or water bottle).
    We are only using the ZED camera.
    Transitions:
    -If arrived at target: DoneState
    -Did not arrive at target: ApproachObjectState
    """

    def on_enter(self, context):
        pass

    def on_exit(self, context):
        pass

    def get_target_pos(self, context) -> Optional[int]:
        object_pos = context.env.current_target_pos()
        return object_pos

    def determine_next(self, context, finished: bool) -> State:
        if finished:
            return state.DoneState()
        else:
            return self
