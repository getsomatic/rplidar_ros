#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <cmath>
#include <cassert>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rplidar.h"
#include <rplidar_ros/config.hh>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace rp::standalone::rplidar;
using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
public:
    explicit PublisherNode(const std::shared_ptr<Config> &config, int channel);

    void publish_scan(rplidar_response_measurement_node_hq_t *nodes,
                      size_t node_count, rclcpp::Time start,
                      double scan_time, bool inverted,
                      float angle_min, float angle_max,
                      float max_distance,
                      std::string frame_id);;

    bool getRPLIDARDeviceInfo(RPlidarDriver * drv, std::string &sn);

    bool checkRPLIDARHealth(RPlidarDriver * drv);

    bool stop_motor(std_srvs::srv::Empty::Request::SharedPtr req,
                    std_srvs::srv::Empty::Response::SharedPtr res);

    bool start_motor(std_srvs::srv::Empty::Request::SharedPtr req,
                     std_srvs::srv::Empty::Response::SharedPtr res);

    static float getAngle(const rplidar_response_measurement_node_hq_t& node);

    std::string GetPort(std::string name, int number);

    void spin();

private:

    std::string serial_port;
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;
    bool angle_compensate = true;
    float max_distance = 8.0;
    int angle_compensate_multiple = 1;//it stand of angle compensate at per 1 degree
    std::string scan_mode;
    std::string portName;
    int portNumber;
    u_result op_result;

    std::string sn; // get rplidar device info

    RPlidarDriver * drv = nullptr;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Clock clock_;

    rclcpp::Logger log_;
};