#include <rplidar_ros/publisher.hh>
#include <ament_index_cpp/get_package_share_directory.hpp>

PublisherNode::PublisherNode(int channel) : Node("rplidar_" + std::to_string(channel))
        , log_(rclcpp::get_logger("rplidar"+std::to_string(channel))), channel_(channel)
{
    declare_parameter("active");
    auto active = get_parameter("active").as_bool();
    if (!active) {
        RCLCPP_WARN_STREAM(log_, "lidar node " << channel_ << "is not active");
        return;
    }
    InitParamerers();
    timer_ = this->create_wall_timer(20ms, std::bind(&PublisherNode::Spin, this));
}

void PublisherNode::InitParamerers() {
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
        RCLCPP_DEBUG(log_, "rpLidar config loaded successfully");
    }
}

void PublisherNode::Emergency() {
    RCLCPP_ERROR(log_,"emergency");
    try{
        RCLCPP_ERROR(log_,"before stop");
        Stop();
        RCLCPP_ERROR(log_,"after stop");
    } catch(...){
        RCLCPP_ERROR_STREAM(log_, "couldn't stop on emergency. channel: " << channel_);
    }
    if (drv){
        RCLCPP_ERROR(log_,"deleting");
        delete drv;
        drv = nullptr;
    }
    RCLCPP_ERROR(log_,"end emergency");
}

void PublisherNode::Stop() {
    if (drv){
        bool scan = drv->stop(1000);
        bool motor = drv->stopMotor();
        RCLCPP_FATAL(log_, "Shutting down lidar (motor[%d]; scanner[%d])", motor, scan);
        //rclcpp::Rate(1).sleep();
    }
}

PublisherNode::~PublisherNode() {
    RCLCPP_WARN(log_, "Destructing and shutting down rplidar%d", portNumber);
    Emergency();
}

void PublisherNode::Spin() {
    RCLCPP_INFO_STREAM(log_, "spinning: " << channel_);
    if (!Connected()){
        try{
            Connect();
        } catch(...) {
            RCLCPP_ERROR_STREAM(log_, "couldn't connect. try in 5 sec. channel: " << channel_);
            std::this_thread::sleep_for (std::chrono::seconds(5));
        }
    } else {
        try{
            RCLCPP_INFO_STREAM(log_, "Reading data: " << channel_);
            ReadData();
        } catch(...) {
            RCLCPP_ERROR_STREAM(log_, "Error durring data reading. try to reconnect. channel: " << channel_);
            try {
                Emergency();
            } catch(...) {
                RCLCPP_ERROR_STREAM(log_, "Error on emergency stop");
            }
            drv = nullptr;
        }

    }
}

bool PublisherNode::Connected() {
    return drv != nullptr;
}

void PublisherNode::Connect() {
    RCLCPP_INFO(log_,"connecting");
    portNumber = channel_;
    //serial_port = "/dev/ttyUSB*";
    serial_port = "/dev/ttyUSB0";
    scan_mode = "";

    serial_port = GetPort(portName, portNumber);
    RCLCPP_INFO_STREAM(log_,"port  name: " << portName << " num: " << portNumber << " port: " << serial_port);

    // create the driver instance
    drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);

    RCLCPP_WARN(log_, "Checking Driver --");
    if (!drv) {
        RCLCPP_ERROR(log_,"Create Driver fail, exit");
        Emergency();
        return;
    }
    RCLCPP_WARN(log_, "Connecting");
    if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
        RCLCPP_ERROR(log_, "Error, cannot bind to the specified serial port %s.",serial_port.c_str());
        Emergency();
        return;
    }
    RCLCPP_WARN(log_, "getRPLIDARDeviceInfo");
    if (!getRPLIDARDeviceInfo(drv, sn)) {
        Emergency();
        return;
    }
    RCLCPP_WARN(log_, "checkRPLIDARHealth");
    if (!checkRPLIDARHealth(drv)) {
        Emergency();
        return;
    }


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
                RCLCPP_ERROR(log_,"scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
                for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    RCLCPP_INFO(log_,"\t%s: max_distance: %.1f m, Point number: %.1fK",  iter->scan_mode,
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
        RCLCPP_INFO(log_,"%s: current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d", sn.c_str(),  current_scan_mode.scan_mode,
                    current_scan_mode.max_distance, (1000/current_scan_mode.us_per_sample), angle_compensate_multiple);
    }
    else
    {
        RCLCPP_ERROR(log_,"%s: Can not start scan: %08x!", sn.c_str(), op_result);
    }

    using std::placeholders::_1;
    using std::placeholders::_2;
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan"+sn, 1);
    start_motor_service_ = this->create_service<std_srvs::srv::Empty>("start_motor", std::bind(&PublisherNode::start_motor, this, _1, _2));
    stop_motor_service_ = this->create_service<std_srvs::srv::Empty>("stop_motor", std::bind(&PublisherNode::stop_motor, this, _1, _2));


    drv->startMotor();
    RCLCPP_INFO(log_, "Successfully connected. chanel: %d", channel_);
}

void PublisherNode::publish_scan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count, rclcpp::Time start,
                                 double scan_time, bool inverted, float angle_min, float angle_max, float max_distance,
                                 std::string frame_id) {

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
}

bool PublisherNode::getRPLIDARDeviceInfo(RPlidarDriver *drv, std::string &sn) {
    u_result     op_result;
    rplidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);
    if (IS_FAIL(op_result)) {
        if (op_result == RESULT_OPERATION_TIMEOUT) {
            RCLCPP_ERROR(log_,"Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
        } else {
            RCLCPP_ERROR(log_,"Error, unexpected error, code: %x",op_result);
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    char s[100];
    std::stringstream ss;
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
        ss << (int) devinfo.serialnum[pos] << " ";
        sprintf(s + pos * 2, "%02X", devinfo.serialnum[pos]);
    }

    s[32] = 0;
    sn = s;
    printf("\n");
    RCLCPP_INFO(log_,"HWID=%s", ss.str().c_str());
    RCLCPP_INFO(log_,"Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    RCLCPP_INFO(log_,"Hardware Rev: %d",(int)devinfo.hardware_version);
    return true;
}

bool PublisherNode::checkRPLIDARHealth(RPlidarDriver *drv) {
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) {
        RCLCPP_INFO(log_,"RPLidar health status : %d", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            RCLCPP_ERROR(log_,"Error, rplidar internal error detected. Please reboot the device to retry.");
            return false;
        } else {
            return true;
        }

    } else {
        RCLCPP_ERROR(log_,"Error, cannot retrieve rplidar health code: %x", op_result);
        return false;
    }
}

bool
PublisherNode::stop_motor(std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res) {
    if(!drv)
        return false;

    RCLCPP_DEBUG(log_,"Stop motor");
    drv->stop();
    drv->stopMotor();
    return true;
}

bool
PublisherNode::start_motor(std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res) {
    if(!drv)
        return false;
    RCLCPP_DEBUG(log_,"Start motor");
    drv->startMotor();
    drv->startScan(0,1);
    return true;
}

float PublisherNode::getAngle(const rplidar_response_measurement_node_hq_t &node) {
    return node.angle_z_q14 * 90.f / 16384.f;
}

std::string PublisherNode::GetPort(std::string name, int number) {
    FILE *fp;
    auto root = ament_index_cpp::get_package_share_directory("rplidar_ros");
    std::stringstream ss;
    ss << number;
    std::string path = root + "/scripts/get_serial_port.py '" + name + "' " + ss.str();
    fp = popen(path.c_str(), "r");
    if (fp == NULL) {
        RCLCPP_INFO(log_,"failed to run command");
        return "";
    }
    std::string res;
    char s[1035];
    while (fgets(s, sizeof(s)-1, fp) != NULL) {
        res += s;
    }
    pclose(fp);
    return res;
}

void PublisherNode::ReadData() {
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
        RCLCPP_ERROR(log_,"Invalid lidar scan result: %08x!", op_result);
        if (op_result == RESULT_OPERATION_TIMEOUT){
            RCLCPP_ERROR(log_,"lidar operation timeout");
            Emergency();
        }
    }

}




