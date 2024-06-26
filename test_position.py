#!/usr/bin/env python3
import rospy
import re
from brics_actuator.msg import JointPositions, JointValue

def move_gripper(position):
    gripper_position = JointPositions()

    gripper_position.poisonStamp.originator = ''
    gripper_position.poisonStamp.description = ''
    gripper_position.poisonStamp.qos = 1.0

    gripper_position.positions.append(JointValue())
    gripper_position.positions[0].timeStamp.secs = 1
    gripper_position.positions[0].timeStamp.nsecs = 1
    gripper_position.positions[0].joint_uri = "gripper_finger_joint_l"
    gripper_position.positions[0].unit = "m"
    gripper_position.positions[0].value = position

    gripper_position.positions.append(JointValue())
    gripper_position.positions[1].timeStamp.secs = 1
    gripper_position.positions[1].timeStamp.nsecs = 1
    gripper_position.positions[1].joint_uri = "gripper_finger_joint_r"
    gripper_position.positions[1].unit = "m"
    gripper_position.positions[1].value = position

    return gripper_position

def move_arm(j1, j2, j3, j4, j5):
    arm_position = JointPositions()

    arm_position.poisonStamp.originator = ''
    arm_position.poisonStamp.description = ''
    arm_position.poisonStamp.qos = 1.0

    arm_position.positions.append(JointValue())
    arm_position.positions[0].timeStamp.secs = 1
    arm_position.positions[0].timeStamp.nsecs = 1
    arm_position.positions[0].joint_uri = "arm_joint_1"
    arm_position.positions[0].unit = "rad"
    arm_position.positions[0].value = j1

    arm_position.positions.append(JointValue())
    arm_position.positions[1].timeStamp.secs = 1
    arm_position.positions[1].timeStamp.nsecs = 1
    arm_position.positions[1].joint_uri = "arm_joint_2"
    arm_position.positions[1].unit = "rad"
    arm_position.positions[1].value = j2

    arm_position.positions.append(JointValue())
    arm_position.positions[2].timeStamp.secs = 1
    arm_position.positions[2].timeStamp.nsecs = 1
    arm_position.positions[2].joint_uri = "arm_joint_3"
    arm_position.positions[2].unit = "rad"
    arm_position.positions[2].value = j3

    arm_position.positions.append(JointValue())
    arm_position.positions[3].timeStamp.secs = 1
    arm_position.positions[3].timeStamp.nsecs = 1
    arm_position.positions[3].joint_uri = "arm_joint_4"
    arm_position.positions[3].unit = "rad"
    arm_position.positions[3].value = j4

    arm_position.positions.append(JointValue())
    arm_position.positions[4].timeStamp.secs = 1
    arm_position.positions[4].timeStamp.nsecs = 1
    arm_position.positions[4].joint_uri = "arm_joint_5"
    arm_position.positions[4].unit = "rad"
    if j5 != 0:
        arm_position.positions[4].value = j5

    return arm_position

def main():
    arm_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size=10)
    gripper_pub = rospy.Publisher('/arm_1/gripper_controller/position_command', JointPositions, queue_size=10)
    rospy.init_node('talker', anonymous=False)

    home_position = (3, 1.1, -2.55, 1.75, 2.95)
    grasp_position = (1.5, 2, -4.25, 1.15, 2.95)
    fold_position = (0.02, 0.02, -0.02, 0.03, 2.95)

    while not rospy.is_shutdown():
        # arm_position_vector = re.findall(r'\d+\.\d+', input('Enter arm position: '))
        # arm_position_vector = (0.02, 0.02, -0.02, 0.03, 0)
        # arm_position = move_arm(0.02, 0.02, -0.02, 0.03, 0)

        command = input('Enter arm position: ')

        if command == "home":
            arm_position = move_arm(*home_position)
            arm_pub.publish(arm_position)
        elif command == "grasp":
            arm_position = move_arm(*grasp_position)
            arm_pub.publish(arm_position)
        elif command == "fold":
            arm_position = move_arm(*fold_position)
            arm_pub.publish(arm_position)
        elif command == "open":
            gripper_position = move_gripper(0.01)
            gripper_pub.publish(gripper_position)
        elif command == "close":
            gripper_position = move_gripper(0)
            gripper_pub.publish(gripper_position)
        elif command == "exit":
            break

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass