#ifndef AUBO_MOVEIT_CONFIG_MOVEIT_EXECUTOR_HPP
#define AUBO_MOVEIT_CONFIG_MOVEIT_EXECUTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <string>

#include <aubo_msgs/srv/set_pose_stamped_goal.hpp>

class MoveitExecutor
{
public:
    MoveitExecutor(std::shared_ptr<rclcpp::Node>& node) : node_(node)
    {
        planning_group_ = "manipulator";
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_);

        if (!move_group_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create MoveGroupInterface for planning group: %s", planning_group_.c_str());
            throw std::runtime_error("MoveGroupInterface initialization failed");
        }

        set_pose_service_ = node_->create_service<aubo_msgs::srv::SetPoseStampedGoal>(
            "set_end_effector_pose",
            std::bind(&MoveitExecutor::setPoseStampedGoal, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(node_->get_logger(), "MoveitExecutor initialized with planning group: %s", planning_group_.c_str());
    }
    ~MoveitExecutor() {}

    // Example public method to plan and execute a motion
    bool planAndExecute(const geometry_msgs::msg::Pose & target_pose, const double speed_factor = 1.0)
    {
        move_group_->setPoseTarget(target_pose);
        move_group_->setMaxVelocityScalingFactor(speed_factor);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            move_group_->execute(plan);
        }
        return success;
    }

    // Service callback to set the end effector pose
    void setPoseStampedGoal(
        const std::shared_ptr<aubo_msgs::srv::SetPoseStampedGoal::Request> request,
        std::shared_ptr<aubo_msgs::srv::SetPoseStampedGoal::Response> response)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to set end effector pose");
        if (planAndExecute(request->goal.pose, request->speed_factor))
        {
            response->success = true;
            response->message = "Pose set successfully";
            RCLCPP_INFO(node_->get_logger(), "Pose set successfully");
        }
        else
        {
            response->success = false;
            response->message = "Failed to set pose";
            RCLCPP_ERROR(node_->get_logger(), "Failed to set pose");
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::string planning_group_;

    rclcpp::Service<aubo_msgs::srv::SetPoseStampedGoal>::SharedPtr set_pose_service_;
};
#endif // AUBO_MOVEIT_CONFIG_MOVEIT_EXECUTOR_HPP
