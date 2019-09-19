#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

class HiroPanda
{
private:
    ros::NodeHandle n;
    ros::AsyncSpinner spinner;

    std::string PLANNING_GROUP;

    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Subscriber sub_move_target;
    ros::Subscriber sub_error_recover;
    ros::Subscriber sub_gripper_cmd;
    ros::Publisher pub_error_recover;
    ros::Publisher pub_gripper_width;

    bool wait(ros::Duration _timeout);
    // callbacks
    void targetCb(const geometry_msgs::Pose& msg);
    void errorRecoverCb(const std_msgs::Empty& msg);
    void gripperCmdCb(const std_msgs::String& msg);
public:
    HiroPanda(std::string name, std::string group = "panda_arm");
    ~HiroPanda();

    // gripper functions
    // movement
    void gotoPose(geometry_msgs::Pose& target);
};