import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import rosbag

bag = rosbag.Bag('joint_states_1.bag')

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

joint_goal = move_group.get_current_joint_values()
plan_msg = moveit_msgs.msg.RobotTrajectory()
i = 0
new_alpha = []
static_time = None

for topic, msg, t in bag.read_messages(topics=['joint_states']):
    #go to start state
    if i == 0:
        static_time = msg.header.stamp
        joint_goal[0] = msg.position[0]
        joint_goal[1] = msg.position[1]
        joint_goal[2] = msg.position[2]
        joint_goal[3] = msg.position[3]
        joint_goal[4] = msg.position[4]
        joint_goal[5] = msg.position[5]
        joint_goal[6] = msg.position[6]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
    i = i+1
    plan_msg.joint_trajectory.header.seq = 0
    plan_msg.joint_trajectory.joint_names = msg.name[:7]
    alpha = trajectory_msgs.msg.JointTrajectoryPoint()
    alpha.positions = list(msg.position)[:7]
    alpha.velocities = list(msg.velocity)[:7]
    alpha.time_from_start = msg.header.stamp - static_time
    new_alpha.insert(len(new_alpha),None)
    new_alpha[-1] = alpha
    

plan_msg.joint_trajectory.points = new_alpha
print plan_msg

move_group.execute(plan_msg,wait=True)
bag.close()