from dataclasses import dataclass, field

import numpy as np


@dataclass
class Trajectory:
    # Coordinates of the trajectory
    coordinates: np.ndarray
    # Currently tracked coordinate index along trajectory
    cur_pt: int = field(default=0, init=False)

    def get_cur_pt(self) -> np.ndarray:
        # print(self.coordinates)
        # print()
        # print()
        # print()
        return self.coordinates[self.cur_pt]

    def get_next_pt(self) -> np.ndarray:
        if self.cur_pt + 1 < self.coordinates.size:
            return self.coordinates[self.cur_pt + 1]
        return None

    def increment_point(self) -> bool:
        """
        Increments the tracked point in the trajectory, returns true if
        the trajectory is finished
        """
        self.cur_pt += 1
        return self.cur_pt >= len(self.coordinates)
