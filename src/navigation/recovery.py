from typing import Optional

import numpy as np
import rospy
from aenum import Enum

from util.np_utils import rotate_2d
from util.ros_utils import get_rosparam
from util.state_lib.state import State

STOP_THRESH = get_rosparam("recovery/stop_thresh", 0.2)
DRIVE_FWD_THRESH = get_rosparam("recovery/drive_fwd_thresh", 0.34)  # 20 degrees
RECOVERY_DISTANCE = get_rosparam("recovery/recovery_distance", 1.0)
GIVE_UP_TIME = get_rosparam("recovery/give_up_time", 10.0)


class JTurnAction(Enum):
    moving_back: Enum = 0
    j_turning: Enum = 1


class RecoveryState(State):
    waypoint_behind: Optional[np.ndarray]
    current_action: JTurnAction
    start_time: Optional[rospy.Time] = None
    waypoint_calculated: bool

    def reset(self, context) -> None:
        self.waypoint_calculated = False
        self.waypoint_behind = None
        self.current_action = JTurnAction.moving_back
        context.rover.stuck = False
        self.start_time = None

    def on_enter(self, context) -> None:
        self.reset(context)
        self.start_time = rospy.Time.now()

    def on_exit(self, context) -> None:
        self.reset(context)

    def on_loop(self, context) -> State:
        if rospy.Time.now() - self.start_time > rospy.Duration(GIVE_UP_TIME):
            return context.rover.previous_state
        # Making waypoint behind the rover to go backwards
        pose = context.rover.get_pose()
        # if first round, set a waypoint directly behind the rover and command it to
        # drive backwards toward it until it arrives at that point.
        if self.current_action == JTurnAction.moving_back:
            # Only set waypoint_behind once so that it doesn't get overwritten and moved
            # further back every iteration
            if self.waypoint_behind is None:
                dir_vector = -1 * RECOVERY_DISTANCE * pose.rotation.direction_vector()
                self.waypoint_behind = pose.position + dir_vector

            cmd_vel, arrived_back = context.rover.driver.get_drive_command(
                self.waypoint_behind, pose, STOP_THRESH, DRIVE_FWD_THRESH, drive_back=True
            )
            context.rover.send_drive_command(cmd_vel)

            if arrived_back:
                self.current_action = JTurnAction.j_turning  # move to second part of turn
                self.waypoint_behind = None
                context.rover.driver.reset()

        # if second round, set a waypoint off to the side of the rover and command it to
        # turn and drive backwards towards it until it arrives at that point. So it will begin
        # by turning then it will drive backwards.
        if self.current_action == JTurnAction.j_turning:
            if self.waypoint_behind is None:
                dir_vector = pose.rotation.direction_vector()
                # the waypoint will be 45 degrees to the left of the rover behind it.
                dir_vector[:2] = RECOVERY_DISTANCE * rotate_2d(dir_vector[:2], 3 * np.pi / 4)
                self.waypoint_behind = pose.position + dir_vector

            cmd_vel, arrived_turn = context.rover.driver.get_drive_command(
                self.waypoint_behind, pose, STOP_THRESH, DRIVE_FWD_THRESH, drive_back=True
            )
            context.rover.send_drive_command(cmd_vel)

            # set stuck to False
            if arrived_turn:
                return context.rover.previous_state

        return self
