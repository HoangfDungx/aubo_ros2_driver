#include "aubo_moveit_config/moveit_executor.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_executor_node");
    auto executor = std::make_shared<MoveitExecutor>(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}