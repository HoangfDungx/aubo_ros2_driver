#include "aubo_moveit_config/moveit_executor.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveitExecutor>("moveit_executor_node");

    RCLCPP_INFO(node->get_logger(), "MoveitExecutor node started");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}