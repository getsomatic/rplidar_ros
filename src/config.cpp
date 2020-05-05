
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals;

class Config : public rclcpp::Node
{
public:
    explicit Config(): Node("rplidar_config"), log_(rclcpp::get_logger("rplidar_config"))
    {
        InitParamerers();
    }

    std::string NodeName() const {
        return "rplidar";
    }

    std::string portName;
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;
    bool angle_compensate = true;

private:
    void InitParamerers(){
        bool noErrors = true;

        this->declare_parameter("angle_compensate");
        rclcpp::Parameter p;
        noErrors = noErrors & this->get_parameter("angle_compensate", p);
        angle_compensate = p.as_bool();
        RCLCPP_DEBUG(log_, "angle_compensate=%d", angle_compensate);

        this->declare_parameter("frame_id");
        noErrors = noErrors & this->get_parameter("frame_id", frame_id);
        RCLCPP_DEBUG(log_, "frame_id=%s", frame_id.c_str());

        this->declare_parameter("portName");
        noErrors = noErrors & this->get_parameter("portName", portName);
        RCLCPP_DEBUG(log_, "portName=%s", portName.c_str());

        this->declare_parameter("inverted");
        noErrors = noErrors & this->get_parameter("inverted", p);
        inverted = p.as_bool();
        RCLCPP_DEBUG(log_, "inverted=%d", inverted);

        this->declare_parameter("serial_baudrate");
        noErrors = noErrors & this->get_parameter("serial_baudrate",serial_baudrate);
        RCLCPP_DEBUG(log_, "serial_baudrate=%d", serial_baudrate);

        if (!noErrors) {
            RCLCPP_ERROR(log_, "Failed to load default rpLidar configuration!");
            assert(false);
        } else {
            RCLCPP_ERROR(log_, "rpLidar config loaded successfully");
        }
    }

    rclcpp::Logger log_;
};