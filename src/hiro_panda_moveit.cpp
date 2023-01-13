#include <hiro_panda_moveit/hiro_panda_moveit.h>
#include <ros/package.h>

#include <franka_msgs/ErrorRecoveryActionGoal.h>
#include <franka_gripper/MoveActionGoal.h>
#include <franka_gripper/GraspActionGoal.h>

#include <string>
#include <cstdlib>
#include <algorithm>

using namespace std;

HiroPanda::HiroPanda(string name, string group) : n(name), spinner(8), PLANNING_GROUP(group), move_group(group)
{
    sub_move_target = n.subscribe("/hiro/panda/target", 1, &HiroPanda::targetCb, this);
    sub_error_recover = n.subscribe("/hiro/panda/error_recover", 1, &HiroPanda::errorRecoverCb, this);
    sub_gripper_cmd = n.subscribe("/hiro/panda/gripper_cmd", 1, &HiroPanda::gripperCmdCb, this);
    pub_error_recover = n.advertise<franka_msgs::ErrorRecoveryActionGoal>("/franka_control/error_recovery/goal", 1);
    pub_gripper_width = n.advertise<franka_gripper::GraspActionGoal>("/franka_gripper/grasp/goal", 1);

    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setNumPlanningAttempts(16);
    move_group.setPlanningTime(10);
    move_group.setMaxAccelerationScalingFactor(0.4);
    move_group.setMaxVelocityScalingFactor(0.4);

    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    spinner.start();
}

HiroPanda::~HiroPanda()
{
    spinner.stop();
}

bool HiroPanda::wait(ros::Duration _timeout)
{
    ros::Rate r(200);
    ros::Time start = ros::Time::now();

    while(ros::ok())
    {
        ROS_DEBUG("Waiting...");
        if (ros::Time::now() - start > _timeout)
        {
            return true;
        }

        r.sleep();
    }

    return false;
}

void HiroPanda::gotoPose(geometry_msgs::Pose& target)
{
    move_group.setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();
}

void HiroPanda::targetCb(const geometry_msgs::Pose& msg)
{
    move_group.setPoseTarget(msg);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.move();
}

void HiroPanda::errorRecoverCb(const std_msgs::Empty& msg)
{
    franka_msgs::ErrorRecoveryActionGoal recover_msg;
    pub_error_recover.publish(recover_msg);
}

void HiroPanda::gripperCmdCb(const std_msgs::String& msg)
{
    if (msg.data == "open")
    {
        franka_gripper::GraspActionGoal g;
        g.goal.speed = 0.2;
        g.goal.force = 0.1;
        g.goal.width = 1.0;
        pub_gripper_width.publish(g);
    }
    else if (msg.data == "close")
    {
        franka_gripper::GraspActionGoal g;
        g.goal.speed = 0.2;
        g.goal.force = 0.1;
        g.goal.width = 0.0;
        pub_gripper_width.publish(g);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hiro_panda");
    HiroPanda panda("hiro_panda", "panda_arm");
    ros::AsyncSpinner spinner(4);
    ros::waitForShutdown();
    return 0;
}