from __future__ import annotations
from typing import ClassVar
from unicodedata import normalize
from navigation.context import Gate, Post

import numpy as np

from context import Context, Environment
from state import BaseState
from dataclasses import dataclass
from drive import get_drive_command
from util.np_utils import normalized, perpendicular_2d

STOP_THRESH = 0.2
DRIVE_FWD_THRESH = 0.95

@dataclass
class GateTrajectory:

    @classmethod
    def spider_gate_trajectory(cls, approach_distance: float, gate: Gate, rover_position: np.ndarray) -> GateTrajectory:
        """
        Generates the "Spider" path through the gate
        :param approach_distance: distance to the point straight out from the gate that the rover will drive to
        :param gate:    Gate object representing the gate that the rover will drive through
        :param rover_position: position vector of the rover
        :return:            GateTrajectory object containing the coordinates the rover will need to traverse
        """

        #first we get the positions of the two posts
        post1 = gate.post1
        post2 = gate.post2

        center = (post1 + post2) / 2
        post_direction = normalized(post2 - post1)
        perpendicular = perpendicular_2d(post_direction)
        
        possible_approach_points = [approach_distance * perpendicular + center, 
                                    -approach_distance * perpendicular + center]
        
        possible_preparation_points = [(2 * approach_distance * perpendicular) + post1,
                              (2 * approach_distance * perpendicular) + post2,
                              (-2 * approach_distance * perpendicular) + post1,
                              (-2 * approach_distance * perpendicular) + post2]
        
        #get closest prepration point
        prep_distance_to_rover = [np.linalg.norm(point) - rover_position[0:2] for point in possible_preparation_points]
        prep_idx = np.argmin(np.array(prep_distance_to_rover), axis = 0)
        closest_prep_point = possible_preparation_points[prep_idx]

        #get closest approach point (to selected prep point), set other one to victory point
        approach_dist_to_prep = [np.linalg.norm(point - closest_prep_point) for point in possible_approach_points]
        approach_idx = np.argmin(np.array(approach_dist_to_prep), axis = 0)
        closest_approach_point = possible_approach_points[approach_idx]
        victory_point = possible_preparation_points[1 - approach_idx]

        coordinates = [closest_prep_point, closest_approach_point, victory_point]
        coordinates = [c for c in coordinates]
        return GateTrajectory(coordinates)


# class GateState(BaseState):
#     def __init__(
#         self,
#         context: Context,
#     ):
#         super().__init__(
#             context,
#             add_outcomes=["waypoint_traverse", "single_fiducial", "search"],
#         )
#         self.traj = None

#     def evaluate(self, ud):
#         # Check if a path has been generated and its associated with the same
#         # waypoint as the previous one. Generate one if not
#         waypoint = self.context.course.current_waypoint()
#         if self.traj is None or self.traj.fid_id != waypoint.fiducial_id:
#             self.traj = SearchTrajectory.spiral_traj(
#                 self.context.rover.get_pose().position_vector()[0:2],
#                 5,
#                 2,
#                 waypoint.fiducial_id,
#             )

#         # continue executing this path from wherever it left off
#         target_pos = self.traj.get_cur_pt()
#         cmd_vel, arrived = get_drive_command(
#             target_pos,
#             self.context.rover.get_pose(),
#             STOP_THRESH,
#             DRIVE_FWD_THRESH,
#         )
#         if arrived:
#             # if we finish the spiral without seeing the fiducial, move on with course
#             if self.traj.increment_point():
#                 return "waypoint_traverse"

#         self.context.rover.send_drive_command(cmd_vel)
#         # if we see the fiduicial, go to the fiducial state
#         current_waypoint = self.context.course.current_waypoint()
#         if current_waypoint.fiducial_id != Environment.NO_FIDUCIAL and self.context.env.current_fid_pos() is not None:
#             return "single_fiducial"

#         return "search"
