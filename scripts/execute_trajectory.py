import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import rosbag

bag = rosbag.Bag('joint_states.bag')

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

joint_goal = move_group.get_current_joint_values()
plan_msg = []

print bag

for topic, msg, t in bag.read_messages(topics=['joint_states']):
    individual_plan = msg
    print(type(individual_plan))
    plan_msg.append(individual_plan)
    # joint_goal[0] = joints[0]
    # joint_goal[1] = joints[1]
    # joint_goal[2] = joints[2]
    # joint_goal[3] = joints[3]
    # joint_goal[4] = joints[4]
    # joint_goal[5] = joints[5]
    # joint_goal[6] = joints[6]
    # move_group.go(joint_goal, wait=True)
    # move_group.stop()

move_group.execute(plan_msg,wait=True)
bag.close()