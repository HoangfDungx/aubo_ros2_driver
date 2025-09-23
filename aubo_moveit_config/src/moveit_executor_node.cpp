#include "aubo_moveit_config/moveit_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_executor_node");
    auto executor = std::make_shared<MoveitExecutor>(node);

    // // Multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor m_executor;  // allows multiple callbacks in parallel
    m_executor.add_node(node);

    // rclcpp::spin(node);
    m_executor.spin();
    rclcpp::shutdown();
    return 0;
}