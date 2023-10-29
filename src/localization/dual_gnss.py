import numpy as np
import matplotlib.pyplot as plt


def get_heading(antenna_point_A, antenna_point_B):
    latitude_A = np.radians(antenna_point_A[0])
    latitude_B = np.radians(antenna_point_B[0])
    longitude_A = np.radians(antenna_point_A[1])
    longitude_B = np.radians(antenna_point_B[1])

    x = np.cos(latitude_B) * np.sin(longitude_B - longitude_A)
    y = np.cos(latitude_A) * np.sin(latitude_B) - np.sin(latitude_A) * np.cos(latitude_B) * np.cos(
        longitude_B - longitude_A
    )

    bearing = np.arctan2(x, y)
    return bearing


# test_bearing = get_heading(np.array([39.099912, -94.581213]), np.array([38.627089, -90.200203]))
# print(test_bearing)


def spherical_to_cartesian(spherical_coord: np.ndarray, reference_coord: np.ndarray) -> np.ndarray:
    r = 6371000
    x = r * (np.radians(spherical_coord[1]) - np.radians(reference_coord[1])) * np.cos(np.radians(reference_coord[0]))
    y = r * (np.radians(spherical_coord[0]) - np.radians(reference_coord[0]))
    z = 0
    return np.array([x, y, z])


P_1 = spherical_to_cartesian(np.array([39.099912, -94.581213]), np.array([42.293195, -83.7096706]))

P_2 = spherical_to_cartesian(np.array([38.627089, -90.200203]), np.array([42.293195, -83.7096706]))

result = P_1 - P_2

plt.scatter([P_1[0], P_2[0], result[0]], [P_1[1], P_2[1], result[1]])
plt.plot(
    [P_1[0], P_2[0]],
    [
        P_1[1],
        P_2[1],
    ],
)

plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.grid(True)
plt.show()
