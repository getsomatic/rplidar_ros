#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rplidar_ros/publisher.hh>

// sudo chmod u=rwx,g=,o= ~/development/bcr_ws/install/rplidar_ros/share/rplidar_ros/scripts/get_serial_port.py

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
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
    while (!node3->Ready()){
        r.sleep();
    }

    RCLCPP_WARN(log_, "Starting 4");
    auto node4 = std::make_shared<PublisherNode>(4);
    RCLCPP_WARN(log_, "Started 4\n");

    rclcpp::Rate rate(1000);
    while (rclcpp::ok()) {
        node1->Spin();
        executor.spin_node_some(node1);
        rate.sleep();
        node2->Spin();
        executor.spin_node_some(node2);
        rate.sleep();
        node3->Spin();
        executor.spin_node_some(node3);
        rate.sleep();
        node4->Spin();
        executor.spin_node_some(node4);
        rate.sleep();
    }

    /*executor.add_node(node1);
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
        node1->Stop();
        node2->Stop();
        node3->Stop();
        node4->Stop();
    });*/

    executor.spin();
    rclcpp::shutdown();
    return 0;
}