#ifndef __LIDARSENSOR_HPP
#define __LIDARSENSOR_HPP

#include <chrono>
#include "ldlidar_driver.h"

class LidarSensor {
public:
    LidarSensor()
        : port_name_("/dev/lidar"), serial_baudrate_(230400), type_name_(ldlidar::LDType::LD_19) {
        node_ = std::make_unique<ldlidar::LDLidarDriver>();
    }

    ~LidarSensor() {
        node_->Stop();
    }

    bool InitializeLidar() {
        try {
            node_->RegisterGetTimestampFunctional(std::bind(&LidarSensor::GetSystemTimeStamp, this));
            node_->EnableFilterAlgorithnmProcess(true);

            node_->Start(type_name_, port_name_, serial_baudrate_, ldlidar::COMM_SERIAL_MODE);

            if (node_->WaitLidarCommConnect(3500)) {
                LDS_LOG_INFO("ldlidar communication is normal.", "");
                return true;
            } else {
                LDS_LOG_ERROR("ldlidar communication is abnormal.", "");
                node_->Stop();
                return false;
            }
        } catch (const std::exception& e) {
            LDS_LOG_ERROR("Exception during Lidar initialization: %s", e.what());
            node_->Stop();
            return false;
        }
    }

    std::pair<float, float> RunLidar(int max_distance_obstacle) {
        auto start_time = std::chrono::high_resolution_clock::now();

        ldlidar::Points2D laser_scan_points;
        std::pair<float, float> pair_data;
        float angle_lidar;
        float distance_lidar;
        
        double minDistance = std::numeric_limits<double>::infinity();
        double minAngle = 0.0;

        try {
            if(ldlidar::LDLidarDriver::IsOk()) {
                switch (node_->GetLaserScanData(laser_scan_points, 1500)) {
                    case ldlidar::LidarStatus::NORMAL: {
                        double lidar_scan_freq = 0;
                        node_->GetLidarScanFreq(lidar_scan_freq);

                        for (const auto& point : laser_scan_points) {

                            angle_lidar = point.angle;
                            distance_lidar = point.distance / 1000.0f;

                            if(distance_lidar > 0.05) {
                                if (angle_lidar > 180 && angle_lidar <= 360) {
                                    angle_lidar = angle_lidar - 360;
                                }

                                if (distance_lidar < minDistance) {
                                    minDistance = distance_lidar;
                                    minAngle = -angle_lidar;
                                }
                                pair_data = {minAngle, minDistance};
                            }
                        }

                        if(pair_data.second < minDistance || pair_data.second > max_distance_obstacle) {
                            pair_data = {0, 0};
                        }

                        break;
                    } case ldlidar::LidarStatus::DATA_TIME_OUT: {
                        throw std::runtime_error("ldlidar publish data is time out, please check your lidar device.");;
                    } default: {
                        break;
                    }
                }

                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

                usleep((1000 * 100) - duration.count());  // Sleep 100ms == 10Hz
            }
        } catch (const std::exception& e) {
            node_->Stop();
            std::cerr << "Exception during Lidar operation: " << e.what() << std::endl;
            throw std::runtime_error("[serialCom error] Error reading data from serial port: ");
        }

        return pair_data;
    }

private:
    uint64_t GetSystemTimeStamp() {
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
            std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
        auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
        return static_cast<uint64_t>(tmp.count());
    }

    const std::string port_name_;
    const uint32_t serial_baudrate_;
    const ldlidar::LDType type_name_;
    std::unique_ptr<ldlidar::LDLidarDriver> node_;
    
};

#endif //__LIDARSENSOR_HPP