#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion
import sensor_msgs.point_cloud2 as pc2
import rasterio


class TerrainParticleFilter:
    def __init__(self, terrain_filepath: str, num_particles: int, sigma_x: float, sigma_theta: float, initial_pose: np.array = None) -> None:
        self.terrain_map = self.load_terrain_map(terrain_filepath)
        self.particles = self.init_particles(num_particles, initial_pose)
        self.sigma_x = sigma_x
        self.sigma_theta = sigma_theta

    def tf_matrix(self, pose: np.array) -> np.array:
        """
        Compute the transformation matrix for a given pose.

        :param pose: np array containing pose [x, y, theta]
        :returns: 3x3 np array transformation matrix
        """
        x, y, theta = pose
        return np.array(
            [[np.cos(theta), -np.sin(theta), x], [np.sin(theta), np.cos(theta), y], [0, 0, 1]]
        )

    def load_terrain_map(self, path: str) -> np.array:
        """
        Load terrain point cloud from a raster file.

        :param path: path to raster file
        :returns: NxMx3 np array of terrain points [x, y, z]
        """
        length = 50
        width = 50
        height = 4.820803273566
        
        with rasterio.open(path) as f:
            terrain_map = f.read().squeeze()
        p_length = length / terrain_map.shape[0]
        p_width = width / terrain_map.shape[1]
        # print(f"vmin: {np.min(terrain_map)}, vmax: {np.max(terrain_map)}")
        
        points = np.empty((terrain_map.shape[0], terrain_map.shape[1], 3))
        for i in range(terrain_map.shape[0]):
            for j in range(terrain_map.shape[1]):
                x = (i + 0.5) * p_length - length / 2
                y = (j + 0.5) * p_width - width / 2
                z = terrain_map[i, j] #* height
                points[i, j] = np.array([x, y, z])

        return points 
        # fig = plt.figure()
        # ax = fig.add_subplot(projection="3d")
        # ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], marker=".", s=0.1)
        # plt.show()
    
    def init_particles(self, num_particles: int, initial_pose: np.array) -> np.array:
        """
        Initialize the particles with a gaussian distribution around the initial pose estimate.

        :param num_particles: number of particles
        :param initial_pose: np array containing initial pose estimate [x, y, theta]
        :returns: Nx4 np array of particles [x, y, theta, weight]
        """
        sigma = 0.0
        poses = np.random.default_rng().normal(initial_pose, sigma, (num_particles, 3))
        return np.hstack((poses, np.ones((num_particles, 1)) / num_particles))

    # prediction step
    def predict(self, u: np.array, dt: float) -> None:
        """
        Update the particle positions based on the input command.

        :param u: np array of commanded velocities [v_x, v_y, omega]
        :param dt: time step
        """
        # integrate velocities to get change in pose in the frame of the previous pose
        x2, y2, theta2 = u * dt
        # print(dt)
        # print(x2, y2, theta2)

        for p in self.particles:
            x1, y1, theta1, _ = p

            # add noise to the deltas
            x2n = x2 + np.random.default_rng().normal(0, self.sigma_x)
            theta2n = theta2 + np.random.default_rng().normal(0, self.sigma_theta)

            dx = x2n * np.cos(theta1) - y2 * np.sin(theta1)
            dy = y2 * np.cos(theta1) + x2n * np.sin(theta1)
            dtheta = theta2n
            p += np.array([dx, dy, dtheta, 0])
            p[2] %= 2 * np.pi

    def update(self, z: np.array) -> None:
        """
        Update the particle weights based on the IMU sensor measurement.
        TODO: use full orientation instead of just gravity vector

        :param z: np array of IMU accelerometer measurement [x, y, z]
        """
        ...


class TerrainParticleFilterNode:
    def __init__(self) -> None:
        self.terrain_filepath = "terrain.tif"
        self.num_particles = 1
        self.initialized = False
        self.last_time = rospy.Time.now()
        self.gt_count = 2000
        self.pf_list = []
        self.gt_list = []
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        rospy.Subscriber("/ground_truth", Odometry, self.ground_truth_callback)
        self.terrain_pub = rospy.Publisher("/terrain_map", PointCloud2, queue_size=10)
        rospy.on_shutdown(self.plot_data)

    def publish_terrain_map(self) -> None:
        """
        Publish the terrain map as a point cloud.
        """
        if not self.initialized:
            return
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        pc = pc2.create_cloud_xyz32(header, self.pf.terrain_map)
        self.terrain_pub.publish(pc)
        
    def cmd_callback(self, msg: Twist) -> None:
        if not self.initialized:
            return

        cur_time = rospy.Time.now()
        dt = (cur_time - self.last_time).to_sec()
        self.pf.predict(np.array([msg.linear.x, msg.linear.y, msg.angular.z]), dt)
        self.last_time = cur_time

    def ground_truth_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        theta %= 2 * np.pi

        if not self.initialized:
            self.pf = TerrainParticleFilter(
                self.terrain_filepath, self.num_particles, 0.0, 0.0, np.array([p.x, p.y, theta])
            )
            self.initialized = True
            print(np.array([p.x, p.y, theta]))

        if self.gt_count < 100:
            self.gt_count += 1
            return
        # self.pf_list = [self.pf.particles[:, :2]]
        self.pf_list.append(np.copy(self.pf.particles))
        self.gt_list.append(np.array([p.x, p.y, theta]))
        # self.gt_list = [np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])]
        self.gt_count = 0

    def plot_data(self) -> None:
        self.pf_data = np.vstack(self.pf_list)
        self.gt_data = np.vstack(self.gt_list)
        print(self.pf_data.shape)
        print(self.gt_data.shape)
        print(self.pf_data)
        # self.gt_data -= self.gt_data[0]
        fig, axs = plt.subplots(2, 1)
        axs[0].plot(self.pf_data[:, 0], self.pf_data[:, 1], "r-", label="Particle Filter")
        axs[0].plot(self.gt_data[:, 0], self.gt_data[:, 1], "b-", label="Ground Truth")

        axs[1].plot(self.pf_data[:, 2], "r-", label="Particle Filter")
        axs[1].plot(self.gt_data[:, 2], "b-", label="Ground Truth")
        plt.legend()
        plt.show()


if __name__ == "__main__":
    rospy.init_node("terrain_particle_filter")
    n = TerrainParticleFilterNode()
    while not rospy.is_shutdown():
        n.publish_terrain_map()
        rospy.sleep(0.1)
    # rospy.spin()
