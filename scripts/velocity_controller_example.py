#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from franka_gripper.msg import GraspActionGoal
from sensor_msgs.msg import JointState

pub_vel = rospy.Publisher('/hiro/vel_cmd', Twist, queue_size=1)
pub_grasp = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1)

def stop():
    global pub_vel
    global pub_grasp
    # stop
    print('stopping')
    t = Twist()
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = 0
    t.linear.x = 0
    t.linear.y = 0
    t.linear.z = 0
    pub_vel.publish(t)
    g = GraspActionGoal()
    g.goal.width = 1.0
    g.goal.speed = 0.2
    g.goal.force = 0.1
    pub_grasp.publish(g)


def getPosition(pos):
    print(pos.position[0], pos.position[1])


def pub_vel_cmd():
    global pub_vel
    global pub_grasp
    rospy.Subscriber("/franka_gripper/joint_states", JointState, getPosition)
    rospy.init_node('velocity_controller_example', anonymous=True)
    direction = 1
    while not rospy.is_shutdown():
        # velocity command
        t = Twist()
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = 0
        t.linear.x = 0
        t.linear.y = 0
        t.linear.z = 0.05 if direction == 1 else -0.05
        direction = 1 if direction == 0 else 0
        print(t.linear.z, direction)
        pub_vel.publish(t)
        # gripper
        g = GraspActionGoal()
        g.goal.width = 1.0 if direction == 0 else 0
        g.goal.speed = 0.2
        g.goal.force = 0.1
        pub_grasp.publish(g)
        rospy.rostime.wallsleep(1)


if __name__ == '__main__':
    try:
        rospy.on_shutdown(stop)
        pub_vel_cmd()
    except rospy.ROSInterruptException:
        pass
