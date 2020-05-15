#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rplidar_ros/publisher.hh>

// You should do "sudo chmod u=rwx,g=,o= /home/alex/development/dev_ws/src/rplidar_ros/scripts/get_serial_port.py" for this to work properly!!!

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto config = std::make_shared<Config>();

    auto node1 = std::make_shared<PublisherNode>(config, 1);
    executor.add_node(node1);

    auto node2 = std::make_shared<PublisherNode>(config, 2);
    executor.add_node(node2);

    // Third lidar doesn't work.
/*
    auto node3 = std::make_shared<PublisherNode>(3);
    executor.add_node(node3);
*/

    executor.spin();
    return 0;
}