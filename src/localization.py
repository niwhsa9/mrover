
import rospy
from sensor_msgs.msg import NavSatFix


def gps_callback(data):
    print(data)


def main():
    rospy.init_node("gps_to_odom")
    rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)

    rospy.spin()


main()
