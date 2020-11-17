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
    auto node1 = std::make_shared<PublisherNode>("fron_left");
    auto node2 = std::make_shared<PublisherNode>("front_right");
    auto node3 = std::make_shared<PublisherNode>("back_left");
    auto node4 = std::make_shared<PublisherNode>("back_right");
    executor.add_node(node1);
    executor.add_node(node2);
    executor.add_node(node3);
    executor.add_node(node4);

    rclcpp::on_shutdown([&]{
        RCLCPP_FATAL(log_, "SHUTDOWN");
        executor.cancel();
        executor.remove_node(node1);
        executor.remove_node(node2);
        executor.remove_node(node3);
        executor.remove_node(node4);
        node1 = nullptr;
        node2 = nullptr;
        node3 = nullptr;
        node4 = nullptr;
    });

    executor.spin();
    rclcpp::shutdown();
    return 0;
}