
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals;

class Config : public rclcpp::Node
{
public:
    explicit Config();

    std::string NodeName() const;

    std::string portName;
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;
    bool angle_compensate = true;

private:
    void InitParamerers();

    rclcpp::Logger log_;
};