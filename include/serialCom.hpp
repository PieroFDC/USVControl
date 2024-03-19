#ifndef SERIALCOM_HPP
#define SERIALCOM_HPP

#include <iostream>
#include <string>
#include <thread>
#include <iomanip>
#include "serialib/serialib.hpp"

#define SERIAL_PORT "/dev/ft232"
#define BAUD_RATE 2000000

struct SensorDataInput {
    double lat = -999.00;
    double lon = -999.00;
    double velocity = -999.00;
    double course = -999.00;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    int calibration = 0;
    std::string nrf;
    int sonic = 0;
    double volt = 0;
    int pwml = 0;
    int pwmr = 0;
};

struct SensorDataOutput {
    int pwml = 0;
    int pwmr = 0;
    double cameraYaw = 0.0f;
    int numWaypoint = 0;
    std::string nrf;
};

class SerialCommunication {
public:
    SerialCommunication() {

        char errorOpening = serial_.openDevice(SERIAL_PORT, BAUD_RATE);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        if (errorOpening == 1) {
            std::cout << "Successful connection to " << SERIAL_PORT << std::endl;
        } else {
            throw std::runtime_error("[serialib error] Error al abrir el puerto.");
        }
    }

    ~SerialCommunication() {
        serial_.closeDevice();
    }

    void sendData(const SensorDataOutput& sensor_data_output) {
        try {
            std::ostringstream oss;
            oss << '<' << sensor_data_output.pwml << ',' << sensor_data_output.pwmr << ',' << sensor_data_output.cameraYaw << ',' << sensor_data_output.nrf << '>';
            std::string data = oss.str();
            serial_.writeString(data.c_str());
            
        } catch (const std::exception& e) {
            std::cerr << "Error sending data: " << e.what() << std::endl;
        }
    }

    SensorDataInput receiveData() {
        char current_char;
        std::string data_from_arduino;

        int timeout_read;

        try {
            while(true) {
                timeout_read = serial_.readChar(&current_char, 500);

                if(timeout_read != 1) {
                    throw std::runtime_error("[serialCom error] Error reading data from serial port.");
                }

                if (current_char == '<') {
                    break;
                }
                
            }

            while(true) {
                timeout_read = serial_.readChar(&current_char, 500);

                if(timeout_read != 1) {
                    throw std::runtime_error("No more timeout for data reading.");
                }
    
                if (current_char == '>') {
                    break;
                    
                } else {
                    data_from_arduino += current_char;
                }
            }

        } catch (const std::exception& e) {
            std::cerr << "Error receiving data: " << e.what() << std::endl;
            throw std::runtime_error("[serialCom error] Error reading data from serial port.");
        }

        SensorDataInput sensor_data_inp;

        std::cout << data_from_arduino << std::endl << std::endl;

        std::istringstream iss(data_from_arduino);
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.lat;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.lon;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.velocity;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.course;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.yaw;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.pitch;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.roll;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.calibration;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        std::getline(iss, sensor_data_inp.nrf, ',');
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.sonic;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.volt;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.pwml;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.pwmr;

        data_from_arduino.clear();

        return sensor_data_inp;
    }

private:
    serialib serial_;
};

#endif //SERIALCOM_HPP