// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <hiro_panda_moveit/cartesian_velocity_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

using namespace std;

namespace hiro_panda_moveit
{

bool CartesianVelocityController::init(hardware_interface::RobotHW *robot_hardware,
                                       ros::NodeHandle &node_handle)
{
    double acceleration_scale_param = 0.1;
    node_handle.getParam("/hiro/acceleration_scale", acceleration_scale_param);
    if (acceleration_scale_param > 0 && acceleration_scale_param < 1)
    {
        acceleration_scale = acceleration_scale_param;
    }
    else
    {
        acceleration_scale = 0.1;
    }
    sub_target_vel = node_handle.subscribe("/hiro/vel_cmd", 10, &CartesianVelocityController::targetVelCb, this,
                                           ros::TransportHints().reliable().tcpNoDelay());

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
        ROS_ERROR("CartesianVelocityController: Could not get parameter arm_id");
        return false;
    }

    velocity_cartesian_interface_ = robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
    if (velocity_cartesian_interface_ == nullptr)
    {
        ROS_ERROR(
            "CartesianVelocityController: Could not get Cartesian velocity interface from "
            "hardware");
        return false;
    }
    try
    {
        velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
            velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
        ROS_ERROR_STREAM(
            "CartesianVelocityController: Exception getting Cartesian handle: " << e.what());
        return false;
    }

    auto *state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
        ROS_ERROR("CartesianVelocityController: Could not get state interface from hardware");
        return false;
    }
    try
    {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
        ROS_ERROR_STREAM(
            "CartesianImpedanceExampleController: Exception getting state handle from interface: "
            << ex.what());
        return false;
    }

    return true;
}

void CartesianVelocityController::starting(const ros::Time & /* time */)
{
    _target.angular.x = 0.0;
    _target.angular.y = 0.0;
    _target.angular.z = 0.0;
    _target.linear.x = 0.0;
    _target.linear.y = 0.0;
    _target.linear.z = 0.0;
    last_O_dP_EE_c.fill(0);
    last_O_ddP_EE_c.fill(0);
}

void CartesianVelocityController::update(const ros::Time & /* time */,
                                         const ros::Duration &period)
{
    std::array<double, 6> command = {{_target.linear.x, _target.linear.y, _target.linear.z,
                                      _target.angular.x, _target.angular.y, _target.angular.z}};
    std::array<double, 6> limited_command =
        franka::limitRate(franka::kMaxTranslationalVelocity,
                          franka::kMaxTranslationalAcceleration * acceleration_scale,
                          franka::kMaxTranslationalJerk,
                          franka::kMaxRotationalVelocity,
                          franka::kMaxRotationalAcceleration * acceleration_scale,
                          franka::kMaxRotationalJerk,
                          command, last_O_dP_EE_c, last_O_ddP_EE_c);
    velocity_cartesian_handle_->setCommand(limited_command);
    franka::RobotState robot_state = state_handle_->getRobotState();
    last_O_dP_EE_c = robot_state.O_dP_EE_c;
    last_O_ddP_EE_c = robot_state.O_ddP_EE_c;
}

void CartesianVelocityController::stopping(const ros::Time & /*time*/)
{
    // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
    // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
    // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void CartesianVelocityController::targetVelCb(const geometry_msgs::Twist &target)
{
    _target.angular.x = target.angular.x;
    _target.angular.y = target.angular.y;
    _target.angular.z = target.angular.z;
    _target.linear.x = target.linear.x;
    _target.linear.y = target.linear.y;
    _target.linear.z = target.linear.z;
}

} // namespace hiro_panda_moveit

PLUGINLIB_EXPORT_CLASS(hiro_panda_moveit::CartesianVelocityController,
                       controller_interface::ControllerBase)
