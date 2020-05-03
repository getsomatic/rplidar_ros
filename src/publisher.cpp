
#include <chrono>
#include <functional>
#include <string>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rplidar.h"
#include <cmath>
#include <cassert>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace rp::standalone::rplidar;
using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode(): Node("rplidar")
    {
        using std::placeholders::_1;
        using std::placeholders::_2;
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        start_motor_service_ = this->create_service<std_srvs::srv::Empty>("start_motor", std::bind(&PublisherNode::start_motor, this, _1, _2));
        stop_motor_service_ = this->create_service<std_srvs::srv::Empty>("stop_motor", std::bind(&PublisherNode::stop_motor, this, _1, _2));
        timer_ = this->create_wall_timer(500ms, std::bind(&PublisherNode::spin, this));


        portName = "CP2102 USB to UART Bridge Controller";
        portNumber = 1;
        serial_port = "/dev/ttyUSB0";
        serial_baudrate = 115200;
        frame_id = "laser";
        inverted = false;
        angle_compensate = true;
        scan_mode = "";

        printf("Serial bef=%s\n", serial_port.c_str());
        //serial_port = GetPort(portName, portNumber);
        printf("Serial aft=%s\n", serial_port.c_str());

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rplidar"),"port  name: " << portName << " num: " << portNumber << " port: " << serial_port);

        // create the driver instance
        drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);

        if (!drv) {
            RCLCPP_ERROR(rclcpp::get_logger("rplidar"),"Create Driver fail, exit");
            assert(false);
        }

        // make connection...
        if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
            RCLCPP_ERROR(rclcpp::get_logger("rplidar"), "Error, cannot bind to the specified serial port %s.",serial_port.c_str());
            RPlidarDriver::DisposeDriver(drv);
            assert(false);
        }

        if (!getRPLIDARDeviceInfo(drv, sn)) {
            assert(false);
        }

        // check health...
        if (!checkRPLIDARHealth(drv)) {
            RPlidarDriver::DisposeDriver(drv);
            assert(false);
        }

        drv->startMotor();

        RplidarScanMode current_scan_mode;
        if (scan_mode.empty()) {
            op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
        } else {
            std::vector<RplidarScanMode> allSupportedScanModes;
            op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

            if (IS_OK(op_result)) {
                _u16 selectedScanMode = _u16(-1);
                for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    if (iter->scan_mode == scan_mode) {
                        selectedScanMode = iter->id;
                        break;
                    }
                }

                if (selectedScanMode == _u16(-1)) {
                    RCLCPP_ERROR(rclcpp::get_logger("rplidar"),"scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
                    for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                        RCLCPP_ERROR(rclcpp::get_logger("rplidar"),"\t%s: max_distance: %.1f m, Point number: %.1fK",  iter->scan_mode,
                                     iter->max_distance, (1000/iter->us_per_sample));
                    }
                    op_result = RESULT_OPERATION_FAIL;
                } else {
                    op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
                }
            }
        }

        if(IS_OK(op_result))
        {
            //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
            angle_compensate_multiple = (int)(1000*1000/current_scan_mode.us_per_sample/10.0/360.0);
            if(angle_compensate_multiple < 1)
                angle_compensate_multiple = 1;
            max_distance = current_scan_mode.max_distance;
            RCLCPP_INFO(rclcpp::get_logger("rplidar"),"%s: current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d", sn.c_str(),  current_scan_mode.scan_mode,
                        current_scan_mode.max_distance, (1000/current_scan_mode.us_per_sample), angle_compensate_multiple);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rplidar"),"%s: Can not start scan: %08x!", sn.c_str(), op_result);
        }

    }

    void publish_scan(rplidar_response_measurement_node_hq_t *nodes,
                       size_t node_count, rclcpp::Time start,
                       double scan_time, bool inverted,
                       float angle_min, float angle_max,
                       float max_distance,
                       std::string frame_id){

        static int scan_count = 0;
        sensor_msgs::msg::LaserScan scan_msg;

        scan_msg.header.stamp = start;
        scan_msg.header.frame_id = frame_id;
        scan_count++;

        bool reversed = (angle_max > angle_min);
        if ( reversed ) {
            scan_msg.angle_min =  M_PI - angle_max;
            scan_msg.angle_max =  M_PI - angle_min;
        } else {
            scan_msg.angle_min =  M_PI - angle_min;
            scan_msg.angle_max =  M_PI - angle_max;
        }
        scan_msg.angle_increment =
                (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count-1);

        scan_msg.scan_time = scan_time;
        scan_msg.time_increment = scan_time / (double)(node_count-1);
        scan_msg.range_min = 0.15;
        scan_msg.range_max = max_distance;//8.0;

        scan_msg.intensities.resize(node_count);
        scan_msg.ranges.resize(node_count);
        bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
        if (!reverse_data) {
            for (size_t i = 0; i < node_count; i++) {
                float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
                if (read_value == 0.0)
                    scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
                else
                    scan_msg.ranges[i] = read_value;
                scan_msg.intensities[i] = (float) (nodes[i].quality >> 2);
            }
        } else {
            for (size_t i = 0; i < node_count; i++) {
                float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000;
                if (read_value == 0.0)
                    scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
                else
                    scan_msg.ranges[node_count-1-i] = read_value;
                scan_msg.intensities[node_count-1-i] = (float) (nodes[i].quality >> 2);
            }
        }

        publisher_->publish(scan_msg);
    };

    bool getRPLIDARDeviceInfo(RPlidarDriver * drv, std::string &sn)
    {
        u_result     op_result;
        rplidar_response_device_info_t devinfo;

        op_result = drv->getDeviceInfo(devinfo);
        if (IS_FAIL(op_result)) {
            if (op_result == RESULT_OPERATION_TIMEOUT) {
                RCLCPP_ERROR(rclcpp::get_logger("rplidar"),"Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rplidar"),"Error, unexpected error, code: %x",op_result);
            }
            return false;
        }

        // print out the device serial number, firmware and hardware version number..
        printf("RPLIDAR S/N: ");
        char s[100];
        for (int pos = 0; pos < 16 ;++pos) {
            printf("%02X", devinfo.serialnum[pos]);
            sprintf(s + pos * 2, "%02X", devinfo.serialnum[pos]);
        }
        s[32] = 0;
        sn = s;
        printf("\n");
        RCLCPP_INFO(rclcpp::get_logger("rplidar"),"Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
        RCLCPP_INFO(rclcpp::get_logger("rplidar"),"Hardware Rev: %d",(int)devinfo.hardware_version);
        return true;
    }

    bool checkRPLIDARHealth(RPlidarDriver * drv)
    {
        u_result     op_result;
        rplidar_response_device_health_t healthinfo;

        op_result = drv->getHealth(healthinfo);
        if (IS_OK(op_result)) {
            RCLCPP_INFO(rclcpp::get_logger("rplidar"),"RPLidar health status : %d", healthinfo.status);
            if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
                RCLCPP_ERROR(rclcpp::get_logger("rplidar"),"Error, rplidar internal error detected. Please reboot the device to retry.");
                return false;
            } else {
                return true;
            }

        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rplidar"),"Error, cannot retrieve rplidar health code: %x", op_result);
            return false;
        }
    }

    bool stop_motor(std_srvs::srv::Empty::Request::SharedPtr req,
                    std_srvs::srv::Empty::Response::SharedPtr res)
    {
        if(!drv)
            return false;

        RCLCPP_DEBUG(rclcpp::get_logger("rplidar"),"Stop motor");
        drv->stop();
        drv->stopMotor();
        return true;
    }

    bool start_motor(std_srvs::srv::Empty::Request::SharedPtr req,
                     std_srvs::srv::Empty::Response::SharedPtr res)
    {
        if(!drv)
            return false;
        RCLCPP_DEBUG(rclcpp::get_logger("rplidar"),"Start motor");
        drv->startMotor();
        drv->startScan(0,1);
        return true;
    }

    static float getAngle(const rplidar_response_measurement_node_hq_t& node)
    {
        return node.angle_z_q14 * 90.f / 16384.f;
    }

    std::string GetPort(std::string name, int number) {
        FILE *fp;
        std::string root = "/home/alex/development/dev_ws/src/rplidar_ros"; //ros::package::getPath("robot_cleaner_core"); // TODO FIX
        std::stringstream ss;
        ss << number;
        std::string path = root + "/scripts/get_serial_port.py '" + name + "' " + ss.str();
        printf("Opening\n");
        fp = popen(path.c_str(), "r");
        if (fp == NULL) {
            RCLCPP_INFO(rclcpp::get_logger("rplidar"),"failed to run command");
            return "";
        }
        printf("Opening2\n");
        std::string res;
        char s[1035];
        while (fgets(s, sizeof(s)-1, fp) != NULL) {
            res += s;
        }
        pclose(fp);
        return res;
    }

    void spin() {
        rplidar_response_measurement_node_hq_t nodes[360*8];
        size_t   count = _countof(nodes);


        rclcpp::Time start_scan_time;
        rclcpp::Time end_scan_time;
        double scan_duration;

        start_scan_time = clock_.now();
        op_result = drv->grabScanDataHq(nodes, count);
        end_scan_time = clock_.now();
        scan_duration = (end_scan_time - start_scan_time).seconds();

        if (op_result == RESULT_OK) {
            op_result = drv->ascendScanData(nodes, count);
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);
            if (op_result == RESULT_OK) {
                if (angle_compensate) {
                    //const int angle_compensate_multiple = 1;
                    const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
                    int angle_compensate_offset = 0;
                    rplidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
                    memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_hq_t));

                    int i = 0, j = 0;
                    for( ; i < count; i++ ) {
                        if (nodes[i].dist_mm_q2 != 0) {
                            float angle = getAngle(nodes[i]);
                            int angle_value = (int)(angle * angle_compensate_multiple);
                            if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                            for (j = 0; j < angle_compensate_multiple; j++) {
                                angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
                            }
                        }
                    }

                    publish_scan(angle_compensate_nodes, angle_compensate_nodes_count,
                                 start_scan_time, scan_duration, inverted,
                                 angle_min, angle_max, max_distance,
                                 frame_id);
                } else {
                    int start_node = 0, end_node = 0;
                    int i = 0;
                    // find the first valid node and last valid node
                    while (nodes[i++].dist_mm_q2 == 0);
                    start_node = i-1;
                    i = count -1;
                    while (nodes[i--].dist_mm_q2 == 0);
                    end_node = i+1;

                    angle_min = DEG2RAD(getAngle(nodes[start_node]));
                    angle_max = DEG2RAD(getAngle(nodes[end_node]));

                    publish_scan(&nodes[start_node], end_node-start_node +1,
                                 start_scan_time, scan_duration, inverted,
                                 angle_min, angle_max, max_distance,
                                 frame_id);
                }
            } else if (op_result == RESULT_OPERATION_FAIL) {
                // All the data is invalid, just publish them
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);

                publish_scan(nodes, count,
                             start_scan_time, scan_duration, inverted,
                             angle_min, angle_max, max_distance,
                             frame_id);
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rplidar"),"Invalid lidar scan result: %08x!", op_result);
            if (op_result == RESULT_OPERATION_TIMEOUT){
                RCLCPP_ERROR(rclcpp::get_logger("rplidar"),"exiting");
                exit(1);
            }
        }
        //RCLCPP_ERROR(rclcpp::get_logger("rplidar"),"Publishing");
        //rclcpp::spin(this->get_node_base_interface());
    }

private:
    void timer_callback()
    {

    }

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
};