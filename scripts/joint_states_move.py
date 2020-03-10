import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

joint_goal = move_group.get_current_joint_values()
print joint_goal

joint_goal[0] = 0.3246022922292559
joint_goal[1] = 0.3256836928317421
joint_goal[2] = -0.017759848881787103
joint_goal[3] = -2.170569140984866
joint_goal[4] = 0.02027384863591144
joint_goal[5] = 1.9857317305405935
joint_goal[6] = 0.34047426338435566

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()

