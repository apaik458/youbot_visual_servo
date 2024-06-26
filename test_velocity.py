#!/usr/bin/env python3
import rospy
import re
from brics_actuator.msg import JointVelocities, JointValue

# getkey
import rospy
import os
import sys
import termios
import tty
from select import select

def getkey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    return key

def move_gripper(velocity):
    gripper_velocity = JointVelocities()

    gripper_velocity.poisonStamp.originator = ''
    gripper_velocity.poisonStamp.description = ''
    gripper_velocity.poisonStamp.qos = 1.0

    gripper_velocity.velocities.append(JointValue())
    gripper_velocity.velocities[0].timeStamp.secs = 1
    gripper_velocity.velocities[0].timeStamp.nsecs = 1
    gripper_velocity.velocities[0].joint_uri = "gripper_finger_joint_l"
    gripper_velocity.velocities[0].unit = "m"
    gripper_velocity.velocities[0].value = velocity

    gripper_velocity.velocities.append(JointValue())
    gripper_velocity.velocities[1].timeStamp.secs = 1
    gripper_velocity.velocities[1].timeStamp.nsecs = 1
    gripper_velocity.velocities[1].joint_uri = "gripper_finger_joint_r"
    gripper_velocity.velocities[1].unit = "m"
    gripper_velocity.velocities[1].value = velocity

    return gripper_velocity

def move_arm_1(j1):
    arm_velocity = JointVelocities()

    arm_velocity.poisonStamp.originator = ''
    arm_velocity.poisonStamp.description = ''
    arm_velocity.poisonStamp.qos = 1.0

    arm_velocity.velocities.append(JointValue())
    arm_velocity.velocities[0].timeStamp.secs = 1
    arm_velocity.velocities[0].timeStamp.nsecs = 1
    arm_velocity.velocities[0].joint_uri = "arm_joint_1"
    arm_velocity.velocities[0].unit = "s^-1 rad"
    arm_velocity.velocities[0].value = j1

    return arm_velocity

def move_arm_2(j2):
    arm_velocity = JointVelocities()

    arm_velocity.poisonStamp.originator = ''
    arm_velocity.poisonStamp.description = ''
    arm_velocity.poisonStamp.qos = 1.0

    arm_velocity.velocities.append(JointValue())
    arm_velocity.velocities[0].timeStamp.secs = 1
    arm_velocity.velocities[0].timeStamp.nsecs = 1
    arm_velocity.velocities[0].joint_uri = "arm_joint_2"
    arm_velocity.velocities[0].unit = "s^-1 rad"
    arm_velocity.velocities[0].value = j2

    return arm_velocity

def move_arm_3(j3):
    arm_velocity = JointVelocities()

    arm_velocity.poisonStamp.originator = ''
    arm_velocity.poisonStamp.description = ''
    arm_velocity.poisonStamp.qos = 1.0

    arm_velocity.velocities.append(JointValue())
    arm_velocity.velocities[0].timeStamp.secs = 1
    arm_velocity.velocities[0].timeStamp.nsecs = 1
    arm_velocity.velocities[0].joint_uri = "arm_joint_3"
    arm_velocity.velocities[0].unit = "s^-1 rad"
    arm_velocity.velocities[0].value = j3

    return arm_velocity

def move_arm_4(j4):
    arm_velocity = JointVelocities()

    arm_velocity.poisonStamp.originator = ''
    arm_velocity.poisonStamp.description = ''
    arm_velocity.poisonStamp.qos = 1.0

    arm_velocity.velocities.append(JointValue())
    arm_velocity.velocities[0].timeStamp.secs = 1
    arm_velocity.velocities[0].timeStamp.nsecs = 1
    arm_velocity.velocities[0].joint_uri = "arm_joint_4"
    arm_velocity.velocities[0].unit = "s^-1 rad"
    arm_velocity.velocities[0].value = j4

    return arm_velocity

def move_arm_5(j5):
    arm_velocity = JointVelocities()

    arm_velocity.poisonStamp.originator = ''
    arm_velocity.poisonStamp.description = ''
    arm_velocity.poisonStamp.qos = 1.0

    arm_velocity.velocities.append(JointValue())
    arm_velocity.velocities[0].timeStamp.secs = 1
    arm_velocity.velocities[0].timeStamp.nsecs = 1
    arm_velocity.velocities[0].joint_uri = "arm_joint_5"
    arm_velocity.velocities[0].unit = "s^-1 rad"
    arm_velocity.velocities[0].value = j5

    return arm_velocity

def main():
    arm_pub = rospy.Publisher('/arm_1/arm_controller/velocity_command', JointVelocities, queue_size=10)
    gripper_pub = rospy.Publisher('/arm_1/gripper_controller/velocity_command', JointVelocities, queue_size=10)
    rospy.init_node('talker', anonymous=False)

    settings = termios.tcgetattr(sys.stdin)
    key_timeout = rospy.get_param("~key_timeout", 0.5)

    while not rospy.is_shutdown():
        key = getkey(settings, 0.1)

        if key == 'q':
            break

        if key == 'u':
            print('Moving joint 1 forwards')
            arm_pub.publish(move_arm_1(10))
        elif key == 'j':
            print('Moving joint 1 backwards')
            arm_pub.publish(move_arm_1(-10))
        elif key == 'i':
            print('Moving joint 2 forwards')
            arm_pub.publish(move_arm_2(10))
        elif key == 'k':
            print('Moving joint 2 backwards')
            arm_pub.publish(move_arm_2(-10))
        elif key == 'o':
            print('Moving joint 3 forwards')
            arm_pub.publish(move_arm_3(10))
        elif key == 'l':
            print('Moving joint 3 backwards')
            arm_pub.publish(move_arm_3(-10))
        elif key == 'p':
            print('Moving joint 4 forwards')
            arm_pub.publish(move_arm_4(10))
        elif key == ';':
            print('Moving joint 4 backwards')
            arm_pub.publish(move_arm_4(-10))
        elif key == 'n':
            print('Moving joint 5 forwards')
            arm_pub.publish(move_arm_5(10))
        elif key == 'm':
            print('Moving joint 5 backwards')
            arm_pub.publish(move_arm_5(-10))
        else:
            arm_pub.publish(move_arm_1(0))
            arm_pub.publish(move_arm_2(0))
            arm_pub.publish(move_arm_3(0))
            arm_pub.publish(move_arm_4(0))
            arm_pub.publish(move_arm_5(0))

        # gripper_pub.publish(0.1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass