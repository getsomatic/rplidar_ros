#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rplidar_ros/publisher.hh>

// sudo chmod u=rwx,g=,o= ~/development/bcr_ws/install/rplidar_ros/share/rplidar_ros/scripts/get_serial_port.py

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    rclcpp::Rate r(5);

    auto log_ = rclcpp::get_logger("rplidars");

    RCLCPP_WARN(log_, "Starting 1");
    auto node1 = std::make_shared<PublisherNode>(1);
    RCLCPP_WARN(log_, "Started 1\n");
    while (!node1->Ready()){
        r.sleep();
    }

    RCLCPP_WARN(log_, "Starting 2");
    auto node2 = std::make_shared<PublisherNode>(2);
    RCLCPP_WARN(log_, "Started 2\n");
    while (!node2->Ready()){
        r.sleep();
    }

    RCLCPP_WARN(log_, "Starting 3");
    auto node3 = std::make_shared<PublisherNode>(3);
    RCLCPP_WARN(log_, "Started 3\n");

    executor.add_node(node1);
    executor.add_node(node2);
    executor.add_node(node3);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}