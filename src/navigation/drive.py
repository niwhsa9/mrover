from __future__ import annotations
from typing import Tuple, Optional
import numpy as np
import rospy
from enum import Enum

from geometry_msgs.msg import Twist
from util.SE3 import SE3
from util.np_utils import angle_to_rotate
from util.ros_utils import get_rosparam

default_constants = {
    "max_driving_effort": 1.0,
    "min_driving_effort": -1.0,
    "turning_p": 10.0,
    "max_turning_effort": 1.0,
    "min_turning_effort": -1.0,
    "driving_p": 1.0,
}
ODOM_CONSTANTS = get_rosparam("drive/odom", default_constants)
MAP_CONSTANTS = get_rosparam("drive/map", default_constants)


class DriveController:
    _last_angular_error: Optional[float] = None

    class DriveMode(Enum):
        TURN_IN_PLACE = 1
        DRIVE_FORWARD = 2
        STOPPED = 3

    _driver_state: DriveMode = DriveMode.STOPPED

    def reset(self) -> None:
        """
        Resets the drive controller.
        """
        self._driver_state = self.DriveMode.STOPPED
        self._last_angular_error = None

    def _get_state_machine_output(
        self,
        in_odom: bool,
        angular_error: float,
        linear_error: float,
        completion_thresh: float,
        turn_in_place_thresh: float,
    ) -> Tuple[Twist, bool]:
        """
        Gets the state machine `output for a given angular and linear error.
        :param in_odom: whether the angular and linear error are in the odom frame
        :param angular_error: the angular error to the target
        :param linear_error: the linear error to the target
        :param completion_thresh: the threshold for when we are close enough to the target to consider it reached
        :param turn_in_place_thresh: the threshold for when we should switch from turning in place to driving forward
        :return: a tuple of the command to send to the rover and a boolean indicating whether we are at the target
        :modifies: self._driver_state
        """

        # get the correct constants based on whether we are driving in the odom or map frame
        constants = ODOM_CONSTANTS if in_odom else MAP_CONSTANTS
        MAX_DRIVING_EFFORT = constants["max_driving_effort"]
        MIN_DRIVING_EFFORT = constants["min_driving_effort"]
        MAX_TURNING_EFFORT = constants["max_turning_effort"]
        MIN_TURNING_EFFORT = constants["min_turning_effort"]
        TURNING_P = constants["turning_p"]
        DRIVING_P = constants["driving_p"]

        # if we are at the target position, reset the controller and return a zero command
        if abs(linear_error) < completion_thresh:
            self.reset()
            return (Twist(), True)

        if self._driver_state == self.DriveMode.STOPPED:
            # if the drive mode is STOP (we know we aren't at the target) so we must start moving towards it
            # just switch to the TURN_IN_PLACE state for now under the assumption that we need to turn to face the target
            # return a zero command and False to indicate we aren't at the target (and are also not in the correct state to figure out how to get there)
            self._driver_state = self.DriveMode.TURN_IN_PLACE
            return (Twist(), False)

        elif self._driver_state == self.DriveMode.TURN_IN_PLACE:
            # if we are in the TURN_IN_PLACE state, we need to turn to face the target
            # if we are within the turn threshold to face the target, we can start driving straight towards it
            if abs(angular_error) < turn_in_place_thresh:
                self._driver_state = self.DriveMode.DRIVE_FORWARD
                return (Twist(), False)

            # IVT (Intermediate Value Theorem) check. If the sign of the angular error has changed, this means we've crossed through 0 error
            # in order to prevent osciallation, we 'give up' and just switch to the drive forward state
            elif self._last_angular_error is not None and np.sign(self._last_angular_error) != np.sign(angular_error):
                self._driver_state = self.DriveMode.DRIVE_FORWARD
                return (Twist(), False)

            # if neither of those things are true, we need to turn in place towards our target heading, so set the z component of the output Twist message
            else:
                cmd_vel = Twist()
                cmd_vel.angular.z = np.clip(angular_error * TURNING_P, MIN_TURNING_EFFORT, MAX_TURNING_EFFORT)
                return (cmd_vel, False)

        elif self._driver_state == self.DriveMode.DRIVE_FORWARD:
            # if we are driving straight towards the target and our last angular error was inside the threshold
            # but our current error was outside the threshold, this means that we have crossed from an acceptable
            # turning error to an unacceptable turning error and we must switch back into the TURN_IN_PLACE state
            # the reason we don't just check if the current error is outside is because it would undermine the IVT
            # check in the TURN_IN_PLACE state and cause oscillation, checking it this way makes it so that we only
            # switch back into the TURN_IN_PLACE state on the "rising edge" of the turn error
            last_angular_was_inside = (
                self._last_angular_error is not None and abs(self._last_angular_error) < turn_in_place_thresh
            )
            cur_angular_is_outside = abs(angular_error) >= turn_in_place_thresh
            if cur_angular_is_outside and last_angular_was_inside:
                self._driver_state = self.DriveMode.TURN_IN_PLACE
                return (Twist(), False)
            # otherwise we compute a drive command with both a linear and angular component in the Twist message
            else:
                cmd_vel = Twist()
                cmd_vel.linear.x = np.clip(linear_error * DRIVING_P, MIN_DRIVING_EFFORT, MAX_DRIVING_EFFORT)
                cmd_vel.angular.z = np.clip(angular_error * TURNING_P, MIN_TURNING_EFFORT, MAX_TURNING_EFFORT)
                return (cmd_vel, False)
        else:
            raise ValueError(f"Invalid drive state {self._driver_state}")

    def get_drive_command(
        self: DriveController,
        target_pos: np.ndarray,
        rover_pose: SE3,
        completion_thresh: float,
        turn_in_place_thresh: float,
        in_odom: bool = False,
    ) -> Tuple[Twist, bool]:
        """
        Returns a drive command to get the rover to the target position, calls the state machine to do so and updates the last angular error in the process
        :param target_pos: The target position to drive to.
        :param rover_pose: The current pose of the rover.
        :param completion_thresh: The distance threshold to consider the rover at the target position.
        :param turn_in_place_thresh: The angle threshold to consider the rover facing the target position and ready to drive forward towards it.
        :param in_odom: Whether to use odom constants or map constants.
        :return: A tuple of the drive command and a boolean indicating whether the rover is at the target position.
        :modifies: self._last_angular_error
        """

        # get the direction vector of the rover and the target position, zero the Z components of both since our controller only assumes motion and control over the Rover in the XY plane
        rover_dir = rover_pose.rotation.direction_vector()
        rover_dir[2] = 0
        rover_pos = rover_pose.position
        rover_pos[2] = 0
        target_pos[2] = 0
        target_dir = target_pos - rover_pos

        # compute errors
        linear_error = float(np.linalg.norm(target_dir))
        angular_error = angle_to_rotate(rover_dir, target_dir)

        output = self._get_state_machine_output(
            in_odom, angular_error, linear_error, completion_thresh, turn_in_place_thresh
        )

        self._last_angular_error = angular_error
        return output
