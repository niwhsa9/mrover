import numpy as np
import rospy
import tf2_ros
from geometry_msgs import Twist
from util.SE3 import SE3

# Servo data and GPS data
# get tf tree latest map to base link transform, 
# average imu reading or middle imu reading since if you take the difference when the imu has just changed to a new direction it'll end your life
# only correct every X minutes, only correct off of N data points and for a certain amount of time

# xval = 0 -> turning in place
# x velocity is some proportion of what it should be

class GPS_Correction:
    def __init__(self):
        rospy.Subscriber("cmd_vel", Twist, self.velocity_callback)
        self.tf_buffer = tf2_ros.Buffer()

        # Tunable Parameters
        self.time_threshold = 3             # seconds (tune this with the num_points_threshold)
        self.num_points_threshold = 30      # TODO: definitely needs to be tuned
        self.cooling_off_period = 300       # seconds
        self.callback_rate = 10             # Hz
        self.linear_vel_threshold = 0.1     # TODO: definitely need to be tuned
        self.angular_vel_threshold = 0.1    # TODO: definitely need to be tuned
        self.linear_angular_ratio = 10      # TODO: definitely needs to be tuned

        self.gps_points = np.empty([0,3])
        self.current_vel = np.zeros((2,3))    # unit linear and angular velocity vectors
        self.driving_straight = False

        self.last_update_time = 0
        self.last_heading_time = 0

        self.world_frame = rospy.get_param("gps_linearization/world_frame")
        self.rover_frame = rospy.get_param("gps_linearization/rover_frame")
    
    def get_heading_change(self, msg: Twist):
        """
        Updates the heading if either the linear velocity change or angular velocity magnitude is too large
        Returns true if the heading has changed, returns false if it has not
        """
        new_vel = np.array([msg.linear, msg.angular])
        new_vel[0] = new_vel[0] / np.linalg.norm(new_vel[0])    # only normalize the linear component
        self.driving_straight = True    # Assume the robot is driving straight

        if(np.dot(self.current_vel[0], new_vel[0]) > self.linear_vel_threshold):
            self.current_vel = new_vel
            return True
        elif(np.linalg.norm(new_vel[0]) > self.angular_vel_threshold):
            self.current_vel = new_vel
            self.driving_straight = False   # Only not driving straight if the robot is turning
            return True
        else:
            return False

    def velocity_callback(self, msg: Twist):
        if(self.get_heading_change(msg)):
            # TODO: think about when we want to update the heading
            if((rospy.Time.now() - self.last_update_time > self.cooling_off_period) and 
              (self.last_heading_time > self.time_threshold) and (self.gps_points.size > self.num_points_threshold)):
                  self.heading_correction = self.get_heading_correction()
                  self.last_update_time = rospy.Time.now()
            np.delete(self.gps_points, [0,1,2], axis=1)
    
    def get_new_readings(self):
        """
        Tries to read gps data at a rate specified by self.callback_rate
        Assumes that the rover is in a valid driving configuration so these points are be valid for heading correction
        """
        lookup_rate = rospy.Rate(self.callback_rate)    # X Hz callback rate
        while(self.driving_straight):
            try:
                # Get linearized transform and append the position in the world frame to our gps_points array
                transform = SE3.from_tf_tree(self.tf_buffer, self.world_frame, self.rover_frame)
                self.gps_points.append(transform.position, axis=0)      # TODO: might have to reshape transform.position to (1,3)
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            if(self.gps_points.size() > 100 * self.num_points_threshold):   # compute the heading correction if we cross an upper points threshold
                self.get_heading_correction()
                self.last_update_time = rospy.Time.now()

            lookup_rate.sleep()

    def get_heading_correction(self):
        """
        Generates the correction matrix to multiply the IMU heading matrix by
        Assumes all conditions are acceptable to calculate the correction (time driving straight, number of datapoints, etc.)
        """
        heading = np.mean(self.gps_points, axis=2)
        heading_rotation = np.array([[heading[0], heading[1],  0],
                                    [heading[1], -heading[0], 0], 
                                    [0,          0,           1]])
        IMU_rotation = SE3.from_tf_tree(self.tf_buffer, self.world_frame, self.rover_frame)     # TODO: Gives us the last one, we want to look one up from the middle
        correction = np.matmul(np.linalg.inv(IMU_rotation), heading_rotation)
        return correction

def main():
    rospy.init_node("gps_correction")
    gps_correction = GPS_Correction()
    gps_correction.get_new_readings()

if __name__ == "__main__":
    main()