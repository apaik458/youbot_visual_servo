#!/usr/bin/env python3
import rospy
from brics_actuator.msg import JointPositions, JointVelocities, JointValue
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

import time
import threading

import PID

class Servo:
    def __init__(self):
        self.position_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size=10)
        self.velocity_pub = rospy.Publisher('/arm_1/arm_controller/velocity_command', JointVelocities, queue_size=10)
        self.point_sub = rospy.Subscriber('/point_topic', Point, self.callback)
        self.joint_sub = rospy.Subscriber('/arm_1/joint_states', JointState, self.joint_callback)

        self.joint_state = JointState()

        self.timer = threading.Timer(5,self.timeout) # If 5 seconds elapse, call timeout()
        self.timer.start()

        self.pid1 = PID.PID(3, 1.5, 0.1)   # joint 1
        self.pid4 = PID.PID(2, 1.5, 0.1)   # joint 4
        self.pid1.setSampleTime(0.01)
        self.pid4.setSampleTime(0.01)
        self.pid1.SetPoint = 0
        self.pid4.SetPoint = 0
    
    def callback(self, data):
        global timer
        self.timer.cancel()
        self.timer = threading.Timer(0.1,self.timeout)
        self.timer.start()

        self.pid1.update(data.x)
        self.pid4.update(data.y)

        self.velocity_pub.publish(move_arm_velocity(-self.pid1.output, 0, 0, -self.pid4.output, 0))

    def timeout(self):
        self.velocity_pub.publish(move_arm_velocity(0, 0, 0, 0, 0))

        self.pid1.clear()
        self.pid4.clear()
    
    def joint_callback(self, data):
        self.joint_state = data

def move_arm_position(j1, j2, j3, j4, j5):
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

def move_arm_velocity(j1, j2, j3, j4, j5):
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

    arm_velocity.velocities.append(JointValue())
    arm_velocity.velocities[1].timeStamp.secs = 1
    arm_velocity.velocities[1].timeStamp.nsecs = 1
    arm_velocity.velocities[1].joint_uri = "arm_joint_2"
    arm_velocity.velocities[1].unit = "s^-1 rad"
    arm_velocity.velocities[1].value = j2

    arm_velocity.velocities.append(JointValue())
    arm_velocity.velocities[2].timeStamp.secs = 1
    arm_velocity.velocities[2].timeStamp.nsecs = 1
    arm_velocity.velocities[2].joint_uri = "arm_joint_3"
    arm_velocity.velocities[2].unit = "s^-1 rad"
    arm_velocity.velocities[2].value = j3

    arm_velocity.velocities.append(JointValue())
    arm_velocity.velocities[3].timeStamp.secs = 1
    arm_velocity.velocities[3].timeStamp.nsecs = 1
    arm_velocity.velocities[3].joint_uri = "arm_joint_4"
    arm_velocity.velocities[3].unit = "s^-1 rad"
    arm_velocity.velocities[3].value = j4

    arm_velocity.velocities.append(JointValue())
    arm_velocity.velocities[4].timeStamp.secs = 1
    arm_velocity.velocities[4].timeStamp.nsecs = 1
    arm_velocity.velocities[4].joint_uri = "arm_joint_5"
    arm_velocity.velocities[4].unit = "s^-1 rad"
    arm_velocity.velocities[4].value = j5
    if j5 != 0:
        arm_velocity.velocities[4].value = j5

    return arm_velocity

def main():
    servo = Servo()

    rospy.init_node('servo', anonymous=False)

    grasp_position = (1.4, 2, -4.25, 1.15, 2.95)
    arm_position = move_arm_position(*grasp_position)

    t_end = time.time() + 3
    while time.time() < t_end:
        servo.position_pub.publish(arm_position)

    rospy.spin()

if __name__ == '__main__':
    main()