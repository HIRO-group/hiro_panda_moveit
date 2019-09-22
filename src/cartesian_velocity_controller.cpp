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

namespace hiro_panda_moveit
{

bool CartesianVelocityController::init(hardware_interface::RobotHW *robot_hardware,
                                       ros::NodeHandle &node_handle)
{
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
        ROS_ERROR("CartesianVelocityController: Could not get parameter arm_id");
        return false;
    }

    velocity_cartesian_interface_ =
        robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
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

    auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
        ROS_ERROR("CartesianVelocityController: Could not get state interface from hardware");
        return false;
    }

    try
    {
        auto state_handle = state_interface->getHandle(arm_id + "_robot");

        std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        for (size_t i = 0; i < q_start.size(); i++)
        {
            if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1)
            {
                ROS_ERROR_STREAM(
                    "CartesianVelocityController: Robot is not in the expected starting position "
                    "for running this example. Run `roslaunch hiro_panda_moveit "
                    "move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
                    "first.");
                return false;
            }
        }
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
        ROS_ERROR_STREAM(
            "CartesianVelocityController: Exception getting state handle: " << e.what());
        return false;
    }

    return true;
}

void CartesianVelocityController::starting(const ros::Time & /* time */)
{
    elapsed_time_ = ros::Duration(0.0);
}

void CartesianVelocityController::update(const ros::Time & /* time */,
                                         const ros::Duration &period)
{
    elapsed_time_ += period;

    double time_max = 4.0;
    double v_max = 0.05;
    double angle = M_PI / 4.0;
    double cycle = std::floor(
        pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max)) / time_max));
    double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * elapsed_time_.toSec()));
    double v_x = std::cos(angle) * v;
    double v_z = -std::sin(angle) * v;
    std::array<double, 6> command = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
    velocity_cartesian_handle_->setCommand(command);
}

void CartesianVelocityController::stopping(const ros::Time & /*time*/)
{
    // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
    // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
    // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

} // namespace hiro_panda_moveit

PLUGINLIB_EXPORT_CLASS(hiro_panda_moveit::CartesianVelocityController,
                       controller_interface::ControllerBase)
