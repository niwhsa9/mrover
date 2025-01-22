#!/usr/bin/env python3
import rospy
import sys
import math
import time

from mrover.msg import IK
from sensor_msgs.msg import JointState

LINK_BC_LEN = 0.5344417294
LINK_CD_LEN = 0.5531735368
EE_LENGTH = 0.13
DE_LENGTH = 0.044886000454425812
JOINT_B_ANGLE = math.pi / 3 - 0.05
BC_POS_X = LINK_BC_LEN * math.cos(JOINT_B_ANGLE)
BC_POS_Y = LINK_BC_LEN * math.sin(JOINT_B_ANGLE)
JOINT_C_MIN = 1.35
JOINT_C_MAX = 3.19
JOINT_C_OFFSET = 0.16084859151
THETA_MAX = JOINT_B_ANGLE - 1 * JOINT_C_MIN + JOINT_C_OFFSET # this seems to be the correct upper limit
# THETA_MIN = JOINT_B_ANGLE - 1 * JOINT_C_MAX + 0.16084859151 # this seems cooked???
# THETA_MAX = -1 * math.pi / 6
THETA_MIN = JOINT_B_ANGLE - 2.85 + JOINT_C_OFFSET # this seems accurate? (c maxes out at like 2.85 for some reason?)
THETA_MIN += 0.6 # safety margin
THETA_MAX -= 0.2 # safety margin
A = (THETA_MIN + THETA_MAX) / 2
B = (THETA_MAX - THETA_MIN) / 2
Y_MIN = 0.0
Y_MAX = 0.4
JOINT_DE_MIN = -0.97
JOINT_DE_MAX = 0.5 # this should be able to go higher???

joint_angles = {
    "joint_a": 0,
    "joint_b": 0,
    "joint_c": 0,
    "joint_de_pitch": 0,
    "joint_de_roll": 0
}

angles_received = False

def joint_data_callback(msg: JointState):
    global angles_received
    for i in range(len(msg.name)):
        joint_angles[msg.name[i]] = msg.position[i]
    angles_received = True
    # print(joint_angles)

# returns list of positions (relative to arm_base_link) for each arm joint
def fk(angles):
    positions = []
    # this is actually position of joint b (at the base of the arm)
    positions.append([0, angles["joint_a"], 0])
    angle = -1 * angles["joint_b"]
    positions.append([LINK_BC_LEN * math.cos(angle), positions[0][1], LINK_BC_LEN * math.sin(angle)])
    angle -= angles["joint_c"] - JOINT_C_OFFSET
    positions.append([positions[1][0] + LINK_CD_LEN * math.cos(angle), positions[0][1], positions[1][2] + LINK_CD_LEN * math.sin(angle)])
    angle -= angles["joint_de_pitch"] + JOINT_C_OFFSET
    positions.append([positions[2][0] + (EE_LENGTH + DE_LENGTH) * math.cos(angle), positions[0][1], positions[2][2] + (EE_LENGTH + DE_LENGTH) * math.sin(angle)])
    return positions

def main():
    rospy.init_node("test_ik")
    ik_pub = rospy.Publisher("arm_ik", IK, queue_size=1)
    angle_sub = rospy.Subscriber("arm_joint_data", JointState, joint_data_callback)
    # wait until we figure out where the arm currently is
    while not angles_received:
        pass

    print("angles:", joint_angles)
    positions = fk(joint_angles)
    print("positions:", positions)
    
    # t = math.pi / 2 + 1.2
    

    # while True:
    #     y = (Y_MAX + Y_MIN) / 2 + (Y_MAX - Y_MIN) / 2 * math.sin(t)
    #     theta = A + B * math.cos(t)
    #     ee_theta = theta + (JOINT_DE_MAX + JOINT_DE_MIN) / 2 + (JOINT_DE_MAX - JOINT_DE_MIN) / 2 * math.sin(t)

    #     target = IK()
    #     target.target.header.stamp = rospy.Time.now()
    #     target.target.header.frame_id = "arm_base_link" # maybe make relative to joint c to make sure b doesn't move??
    #     target.target.pose.position.x = BC_POS_X + LINK_CD_LEN * math.cos(theta)
    #     target.target.pose.position.y = 0
    #     target.target.pose.position.z = BC_POS_Y + LINK_CD_LEN * math.sin(theta)
    #     target.target.pose.position.x += (DE_LENGTH + EE_LENGTH) * math.cos(ee_theta)
    #     target.target.pose.position.z += (DE_LENGTH + EE_LENGTH) * math.sin(ee_theta)
    #     target.target.pose.orientation.x = ee_theta


    #     print(f"{target.target.pose.position.x}, {target.target.pose.position.y}, {target.target.pose.position.z}")
    #     print(f"pitch: {target.target.pose.orientation.x}")
    #     print(f"t: {t}")
    #     inp = input("Press enter to send")
    #     if (inp == "q"):
    #         return
    #     ik_pub.publish(target)
    #     print("Sent!")
        # time.sleep(2)
        # actual_x = BC_POS_X + LINK_CD_LEN * math.cos(JOINT_B_ANGLE - joint_angles["joint_c"] + 0.16084859151) + (DE_LENGTH + EE_LENGTH) * math.cos(JOINT_B_ANGLE - joint_angles["joint_c"] + 0.16084859151 - joint_angles["joint_de_pitch"])
        # actual_z = BC_POS_Y + LINK_CD_LEN * math.sin(JOINT_B_ANGLE - joint_angles["joint_c"] + 0.16084859151) + (DE_LENGTH + EE_LENGTH) * math.sin(JOINT_B_ANGLE - joint_angles["joint_c"] + 0.16084859151 - joint_angles["joint_de_pitch"])
        # print(f"Actual: {actual_x}, {actual_z}")

        # t += 0.1


if __name__ == "__main__":
    main()
