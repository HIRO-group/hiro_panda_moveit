#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from franka_gripper.msg import GraspActionGoal
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaState
from std_msgs.msg import Int16

pub_vel = rospy.Publisher('/hiro/vel_cmd', Twist, queue_size=1)
pub_grasp = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1)

global ee_x
global ee_y
global ee_z
ee_x = 0
ee_y = 0
ee_z = 0

global x_prox
x_prox = -1 # -1 means that we should not worry about the distance measurment


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

def euclidianDist(waypoint):
    d = [waypoint[0]-ee_x, waypoint[1]-ee_y, waypoint[2]-ee_z]
    return np.sqrt(np.dot(d,d))

def getPosition(pos):
    # print(pos.position[0], pos.position[1])
    pass

def updateEEPosition(state):
    global ee_x
    global ee_y
    global ee_z
    
    base_position = np.array([[0], [0], [0], [1.0]])
    OTEE = [i for i in state.O_T_EE]
    base_to_EE_rotation_matrix = np.array([[OTEE[0], OTEE[4], OTEE[8],  OTEE[12]], 
                                           [OTEE[1], OTEE[5], OTEE[9],  OTEE[13]],
                                           [OTEE[2], OTEE[6], OTEE[10], OTEE[14]],
                                           [OTEE[3], OTEE[7], OTEE[11], OTEE[15]]])
    EE_position =  np.dot(base_to_EE_rotation_matrix, base_position)

    ee_x = EE_position[0][0]
    ee_y = EE_position[1][0]
    ee_z = EE_position[2][0]


def setProximity(distance):
    global x_prox
    if distance.data > 500:
        x_prox = -1
    elif distance.data < 0:
        pass 
    else:
        x_prox = distance.data
        




def pub_vel_cmd():
    global pub_vel
    global pub_grasp
    
    #TODO: Subscribe to proximity sensor
    #TODO: Callback that updates global proximity vector
    #TODO: Apply force as velocity to end-effector
    #TODO: Script publishing fake distances
    #TODO: Test Case: X force 

    rospy.Subscriber("/franka_gripper/joint_states", JointState, getPosition)
    rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, updateEEPosition)
    rospy.Subscriber("proximity", Int16, setProximity)
    rospy.init_node('velocity_controller_example', anonymous=True)
    #[(x,y,z)] 
    waypoints = np.array([[.607039011, -.0000802080501, .486636405],
                          [.607025049, -.0000674236669, .486662449],
                          [0.60690024, -0.27709862,  0.36052209],
                          [.606596384, .0000587383797, .0410076150],
                          [0.6070476, 0.14861584, 0.19329099],
                          [.607105898, .000161195619, .486516718]])

    direction = 1
    vel = 0.25
    waypoint_idx = 0
    while not rospy.is_shutdown():
        # velocity command
        t = Twist()
        # angular is ee orientation velocity
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = 0

        
        
        dist = euclidianDist(waypoints[waypoint_idx])
        if dist < 0.01:
            waypoint_idx = (waypoint_idx + 1) % 6
        print('dist',dist)
        
        cur_pos = np.array([ee_x,ee_y,ee_z])
        
        unitVec = np.subtract(waypoints[waypoint_idx],cur_pos)/dist
        print('unitVec',unitVec)
        
        S = np.sum(np.abs(unitVec))
        velVec = (unitVec/S)*vel 
        print('velVec',velVec)
        x_add = 0
        if not x_prox == -1 and ee_x > 0.3:
            print('x_prox', x_prox)
            ratio = (500.0 - x_prox)/500.0
            print(ratio)
            x_add = 0.2 * ratio

        
        t.linear.x = velVec[0] - x_add
        t.linear.y = velVec[1] 
        t.linear.z = velVec[2]

        pub_vel.publish(t)

        rospy.rostime.wallsleep(0.01)
        
    


if __name__ == '__main__':
    try:
        rospy.on_shutdown(stop)
        pub_vel_cmd()
    except rospy.ROSInterruptException:
        pass
