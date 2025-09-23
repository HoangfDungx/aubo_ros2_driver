#ifndef AUBO_MOVEIT_CONFIG_MOVEIT_EXECUTOR_HPP
#define AUBO_MOVEIT_CONFIG_MOVEIT_EXECUTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <geometric_shapes/shape_operations.h>
#include <memory>
#include <string>

#include <aubo_msgs/srv/set_pose_stamped_goal.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MoveitExecutor
{
public:
    MoveitExecutor(std::shared_ptr<rclcpp::Node> &node) : node_(node)
    {
        planning_group_ = "manipulator";
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Load robot model
        robot_model_loader::RobotModelLoader robot_model_loader(node_->shared_from_this(), "robot_description");
        kinematic_model = robot_model_loader.getModel();
        kinematic_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
        if (!move_group_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create MoveGroupInterface for planning group: %s", planning_group_.c_str());
            throw std::runtime_error("MoveGroupInterface initialization failed");
        }

        // Add table (collision object) below z=0
        addTable();
        move_group_->startStateMonitor();

        service_cb_group = node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        set_pose_service_ = node_->create_service<aubo_msgs::srv::SetPoseStampedGoal>(
            "set_end_effector_pose",
            std::bind(&MoveitExecutor::setPoseStampedGoal, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(), // default options
            service_cb_group);

        RCLCPP_INFO(node_->get_logger(), "MoveitExecutor initialized with planning group: %s", planning_group_.c_str());
    }
    ~MoveitExecutor() {}

    bool planAndExecute(const geometry_msgs::msg::Pose &target_pose,
                        const std::vector<uint8_t> &defined_constraints,
                        const double speed_factor = 1.0)
    {
        move_group_->setPlanningTime(10.0);
        move_group_->setMaxVelocityScalingFactor(speed_factor);

        // Apply constraints
        moveit_msgs::msg::Constraints constraints;
        for (const auto &constraint_type : defined_constraints)
        {
            addConstraints(constraints, constraint_type);
        }
        move_group_->setPathConstraints(constraints);
        move_group_->setStartStateToCurrentState();

        // Define validity callback for collision checking
        auto validity_callback =
            [this](moveit::core::RobotState *state,
                   const moveit::core::JointModelGroup *jmg,
                   const double *ik_solution) -> bool
        {
            state->setJointGroupPositions(jmg, ik_solution);

            collision_detection::CollisionRequest collision_request;
            collision_detection::CollisionResult collision_result;
            planning_scene_->checkCollision(collision_request, collision_result, *state);

            RCLCPP_INFO(node_->get_logger(), "IK solver check collision: %s",
                        collision_result.collision ? "true" : "false");
            return !collision_result.collision; // valid if no collision
        };

        // Get joint model group
        kinematic_state = move_group_->getCurrentState();
        const moveit::core::JointModelGroup *joint_model_group =
            kinematic_model->getJointModelGroup(planning_group_);

        // Seed IK with current joint positions
        std::vector<double> seed_joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, seed_joint_values);
        RCLCPP_INFO(node_->get_logger(), "Seed joint values:");
        for (size_t i = 0; i < seed_joint_values.size(); ++i)
        {
            RCLCPP_INFO(node_->get_logger(), "  Joint %zu: %f", i + 1, seed_joint_values[i]);
        }
        bool found_ik = false;
        for (size_t i = 0; i < 10; ++i)
        {
            found_ik = kinematic_state->setFromIK(
                joint_model_group, target_pose,
                0.2, validity_callback);

            if (found_ik)
                break;
        }

        if (!found_ik)
        {
            RCLCPP_ERROR(node_->get_logger(), "IK not found");
            move_group_->clearPathConstraints();
            return false;
        }

        // Extract the resulting joint values
        std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        move_group_->setJointValueTarget(joint_values);

        RCLCPP_INFO(node_->get_logger(), "Joint values for target pose:");
        for (size_t i = 0; i < joint_values.size(); ++i)
        {
            RCLCPP_INFO(node_->get_logger(), "  Joint %zu: %f", i + 1, joint_values[i]);
        }

        // Plan and execute
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            move_group_->execute(plan);
        }

        // Clear constraints for next motions
        move_group_->clearPathConstraints();

        return success;
    }

    void setPoseStampedGoal(
        const std::shared_ptr<aubo_msgs::srv::SetPoseStampedGoal::Request> request,
        std::shared_ptr<aubo_msgs::srv::SetPoseStampedGoal::Response> response)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to set end effector pose");
        if (planAndExecute(request->goal.pose, request->defined_constraints, request->speed_factor))
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
    void addTable()
    {
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = move_group_->getPlanningFrame();
        table.id = "table";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {2.0, 2.0, 0.05}; // 2m x 2m x 0.05m

        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = 1.0;
        table_pose.position.x = 0.0;
        table_pose.position.y = 0.0;
        table_pose.position.z = -0.025; // Half thickness below z=0

        table.primitives.push_back(primitive);
        table.primitive_poses.push_back(table_pose);
        table.operation = table.ADD;

        planning_scene_interface_->applyCollisionObjects({table});

        RCLCPP_INFO(node_->get_logger(), "Added table (2m x 2m x 0.05m) below z=0 to the planning scene.");
    }

    void addConstraints(moveit_msgs::msg::Constraints &constraints, const uint8_t constraint_type)
    {
        auto current_pose = move_group_->getCurrentPose();
        RCLCPP_INFO(node_->get_logger(), "Current pose: %f, %f, %f, %f, %f, %f, %f",
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z,
                    current_pose.pose.orientation.x,
                    current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z,
                    current_pose.pose.orientation.w);
        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = "ee_link";
        ocm.header.frame_id = move_group_->getPlanningFrame();
        tf2::Quaternion q;
        switch (constraint_type)
        {
        case aubo_msgs::srv::SetPoseStampedGoal::Request::KEEP_EEF_Y_UP:
            ocm.orientation = current_pose.pose.orientation;
            ocm.absolute_x_axis_tolerance = M_PI / 2;
            ocm.absolute_y_axis_tolerance = M_PI / 2;
            ocm.absolute_z_axis_tolerance = M_PI;
            ocm.weight = 1.0;
            break;
        case aubo_msgs::srv::SetPoseStampedGoal::Request::KEEP_EEF_Z_UP:
            ocm.orientation = current_pose.pose.orientation;
            ocm.absolute_x_axis_tolerance = M_PI / 2;
            ocm.absolute_y_axis_tolerance = M_PI / 2;
            ocm.absolute_z_axis_tolerance = M_PI;
            ocm.weight = 1.0;
            break;
        case aubo_msgs::srv::SetPoseStampedGoal::Request::KEEP_EEF_Y_DOWN:
            ocm.orientation = current_pose.pose.orientation;
            ocm.absolute_x_axis_tolerance = M_PI / 2;
            ocm.absolute_y_axis_tolerance = M_PI / 2;
            ocm.absolute_z_axis_tolerance = M_PI;
            ocm.weight = 1.0;
            break;
        case aubo_msgs::srv::SetPoseStampedGoal::Request::KEEP_EEF_Z_DOWN:
            ocm.orientation = current_pose.pose.orientation;
            ocm.absolute_x_axis_tolerance = M_PI / 2;
            ocm.absolute_y_axis_tolerance = M_PI / 2;
            ocm.absolute_z_axis_tolerance = M_PI;
            ocm.weight = 1.0;
            break;

        default:
            break;
        }

        constraints.orientation_constraints.push_back(ocm);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    planning_scene::PlanningScenePtr planning_scene_;
    std::string planning_group_;
    moveit::core::RobotModelPtr kinematic_model;
    moveit::core::RobotStatePtr kinematic_state;

    rclcpp::Service<aubo_msgs::srv::SetPoseStampedGoal>::SharedPtr set_pose_service_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group;
};
#endif // AUBO_MOVEIT_CONFIG_MOVEIT_EXECUTOR_HPP
