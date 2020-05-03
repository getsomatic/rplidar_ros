#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "publisher.cpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}