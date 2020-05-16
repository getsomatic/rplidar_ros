#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rplidar_ros/publisher.hh>

// sudo chmod u=rwx,g=,o= ~/development/bcr_ws/install/rplidar_ros/share/rplidar_ros/scripts/get_serial_port.py

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto node1 = std::make_shared<PublisherNode>(2);
    executor.add_node(node1);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}