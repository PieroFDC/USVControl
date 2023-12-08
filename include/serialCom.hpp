#ifndef __SERIALCOM_HPP
#define __SERIALCOM_HPP

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <thread>

struct SensorData {
    float lat;
    float lon;
    float speed;
    float yaw;
    std::string nrf;
    int sonic;
    float volt;
    int pwml;
    int pwmr;
};

class SerialCommunication {
public:
    SerialCommunication(const std::string& port, unsigned int baud_rate)
        : io_service_(), serial_port_(io_service_, port) {

        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    ~SerialCommunication() {
        serial_port_.close();
    }

    void sendData(const std::string& data) {
        try {
            boost::asio::write(serial_port_, boost::asio::buffer(data));
        } catch (const std::exception& e) {
            std::cerr << "Error sending data: " << e.what() << std::endl;
        }
    }

    SensorData receiveData() {
        char current_char;
        std::string data_from_arduino;

        try {
            while (true) {
                boost::asio::read(serial_port_, boost::asio::buffer(&current_char, 1));
                if (current_char == '<') {
                    break;
                }
            }

            while (true) {
                boost::asio::read(serial_port_, boost::asio::buffer(&current_char, 1));

                if (current_char == '>') {
                    break;
                } else {
                    data_from_arduino += current_char;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error receiving data: " << e.what() << std::endl;
        }

        std::cout << "Datos recibidos: " << data_from_arduino << std::endl;

        SensorData sensor_data;

        std::istringstream iss(data_from_arduino);
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data.lat;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data.lon;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data.speed;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data.yaw;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        std::getline(iss, sensor_data.nrf, ',');
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data.sonic;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data.volt;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data.pwml;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data.pwmr;

        return sensor_data;
    }

private:
    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_port_;
};

#endif //__SERIALCOM_HPP