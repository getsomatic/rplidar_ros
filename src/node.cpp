#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "publisher.cpp"

// You should do "sudo chmod u=rwx,g=,o= /home/alex/development/dev_ws/src/rplidar_ros/scripts/get_serial_port.py" for this to work properly!!!

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node1 = std::make_shared<PublisherNode>("rplidar", 1);
    auto node2 = std::make_shared<PublisherNode>("rplidar", 2);
    executor.add_node(node1);
    executor.add_node(node2);
    executor.spin();
    return 0;
}