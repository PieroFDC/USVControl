#ifndef __SERIALCOM_HPP
#define __SERIALCOM_HPP

#include <iostream>
#include <string>
#include <thread>
#include <iomanip>
#include "serialib/serialib.hpp"

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD_RATE 2000000

struct SensorDataInput {
    float lat;
    float lon;
    float velocity;
    float course;
    float yaw;
    std::string nrf;
    int sonic;
    float volt;
    int pwml;
    int pwmr;
};

struct SensorDataOutput {
    int pwml;
    int pwmr;
    float camera_yaw;
    std::string nrf;
};

class SerialCommunication {
public:
    SerialCommunication() {

        char errorOpening = serial_.openDevice(SERIAL_PORT, BAUD_RATE);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        if (errorOpening == 1) {
            printf ("Successful connection to %s\n",SERIAL_PORT);
        } else {
            throw "[serialib error] Error al abrir el puerto.";
        }
    }

    ~SerialCommunication() {
        serial_.closeDevice();
    }

    void sendData(const SensorDataOutput& sensor_data_output) {
        try {
            std::ostringstream oss;
            oss << '<' << sensor_data_output.pwml << ',' << sensor_data_output.pwmr << ',' << sensor_data_output.camera_yaw << ',' << sensor_data_output.nrf << '>';
            std::string data = oss.str();
            serial_.writeString(data.c_str());
            
        } catch (const std::exception& e) {
            std::cerr << "Error sending data: " << e.what() << std::endl;
        }
    }

    SensorDataInput receiveData() {
        char current_char;
        std::string data_from_arduino;

        int timeout_read = 0;

        try {
            while(true) {
                timeout_read = serial_.readChar(&current_char, 1000);
                std::cout << "while readchar : " << timeout_read << std::endl;

                if(timeout_read != 1) {
                    throw std::runtime_error("[serialCom error] Error reading data from serial port.");
                }

                if (current_char == '<') {
                    break;
                }
                
            }

            while(true) {
                timeout_read = serial_.readChar(&current_char, 1000);

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

        std::istringstream iss(data_from_arduino);
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> std::setprecision(8) >> sensor_data_inp.lat;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> std::setprecision(8) >> sensor_data_inp.lon;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> std::setprecision(5) >> sensor_data_inp.velocity;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> std::setprecision(5) >> sensor_data_inp.course;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> std::setprecision(5) >> sensor_data_inp.yaw;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        std::getline(iss, sensor_data_inp.nrf, ',');
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.sonic;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> std::setprecision(4) >> sensor_data_inp.volt;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.pwml;
        iss.ignore(std::numeric_limits<std::streamsize>::max(), ':');
        iss >> sensor_data_inp.pwmr;

        data_from_arduino.clear();
        // std::cout << "Se recibió Exitosamente..." << std::endl;
        return sensor_data_inp;
    }

private:
    serialib serial_;
};

#endif //__SERIALCOM_HPP