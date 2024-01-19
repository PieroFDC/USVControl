#include "main.hpp"
#include <chrono>
#include <thread>
#include <random>

int main_custom() {
    // float inp; //
    std::pair<float, float> motors;

    // CollectionController collectionController;
    // CourseController courseController;
    ObstacleController obstacleController;

    // motors = collectionController.calculateMotors(20);
    // std::cout << "Motor L: " << motors.first << std::endl << "Motor R: " << motors.second << std::endl;

    // while(true) {
        
    //     motors = collectionController.calculateMotors({0.5, 0.5});
    //     std::cout << "Motor L: " << motors.first << std::endl << "Motor R: " << motors.second << std::endl;
    // }

    // // Ejemplo de uso
    // std::vector<double> inputAngles1 = {45.0, 200.0, 270.0, 360.0};
    // std::vector<double> convertedAngles = convertAngles(inputAngles1);

    // // Imprimir los ángulos convertidos
    // std::cout << "Ángulos convertidos: ";
    // for (double convertedAngle : convertedAngles) {
    //     std::cout << convertedAngle << " ";
    // }
    // std::cout << std::endl;
    

    // // Ejemplo de uso
    // std::vector<double> inputAngles2 = {30.0, 45.0, 60.0, 179, 1};
    // std::vector<double> inputDistances = {0.1, 0.03, 0.08, 0.06, 0.9};

    // // Llamada a la función
    // std::pair<double, double> result = getMinDistanceAngle(inputAngles2, inputDistances);

    // // Imprimir el resultado
    // std::cout << "Ángulo mínimo: " << result.first << ", Distancia mínima: " << result.second << std::endl;


// ///////////////////// NCNN
//    ObjectDetector detector(
//         MODEL_PARAM_PATH.c_str(),
//         MODEL_BIN_PATH.c_str(),
//         352,
//         352
//     ); 

//     cv::VideoCapture cap(0);

//     cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

//     std::pair<float, float> center;

//     // cv::VideoCapture cap("http://192.168.1.38:8080/video");
//     if (!cap.isOpened()) {
//         printf("Error al abrir la cámara.\n");
//         return -1;
//     }

//     while (true) {
//         // Capturar frame de la cámara
//         cv::Mat frame;
//         cap >> frame;

//         if (frame.empty()) {
//             printf("Error al capturar frame de la cámara.\n");
//             break;
//         }

//         center = detector.detectLoop(frame);

//         if(center.first && center.second){
//             std::cout << center.first << " : " << center.second << std::endl;
//         } else {
//             std::cout << "Not Object Detection" << std::endl;
//         }
        
//         // motors = collectionController.calculateMotors(center);
//         // std::cout << "Motor L: " << motors.first << std::endl << "Motor R: " << motors.second << std::endl;

//         // Romper el bucle si se presiona la tecla 'q'
//         if (cv::waitKey(1) == 'q')
//             break;
//     }

//     cap.release();
//     cv::destroyAllWindows();

    ///////////////// SERIAL
    // Inicializar el generador de números aleatorios
    std::random_device rd;
    std::mt19937 gen(rd());

    // Definir el rango
    std::uniform_int_distribution<int> distribution(1100, 1900);

    // Generar dos números aleatorios
    int numero1;
    int numero2;

    SerialCommunication serialComm;

    SensorDataInput sdatainp;
    SensorDataOutput sdataout;

    sdataout.camera_yaw = 50.0;
    sdataout.nrf = "nrfmsg";

    std::cout << std::fixed << std::setprecision(7);

    // bucle principal
    while(true) {
        numero1 = distribution(gen);
        numero2 = distribution(gen);

        sdataout.pwml = numero1;
        sdataout.pwmr = numero2;

        std::string data_to_send = "<" + std::to_string(numero1) + "," + std::to_string(numero2) + ",50.0,nrfmsg>";

        // // // Envía datos al Arduino
        serialComm.sendData(sdataout);
        std::cout << "Datos enviados: " << data_to_send << std::endl;

        // Recibir datos
        while(true) {
            try {
                sdatainp = serialComm.receiveData();   
                break;
            } catch (const std::exception& e) {
                std::cerr << "Error receiving data: " << e.what() << std::endl;
                serialComm.sendData(sdataout);
            }
        }
        
        std::cout << "Lat: " << sdatainp.lat << std::endl;
        std::cout << "Lon: " << sdatainp.lon << std::endl;
        std::cout << "Speed: " << sdatainp.velocity << std::endl;
        std::cout << "Course: " << sdatainp.course << std::endl;
        std::cout << "Yaw: " << sdatainp.yaw << std::endl;
        std::cout << "NRF: " << sdatainp.nrf << std::endl;
        std::cout << "Sonic: " << sdatainp.sonic << std::endl;
        std::cout << "Volt: " << sdatainp.volt << std::endl;
        std::cout << "PWML: " << sdatainp.pwml << std::endl;
        std::cout << "PWMR: " << sdatainp.pwmr << std::endl;

    }

    return 0;
    // std::pair<float, float> lidar_data;
    // LidarSensor lidar_sensor;
    // lidar_sensor.InitializeLidar();
    // while(true) {
    //     lidar_data = lidar_sensor.RunLidar();
    //     std::cout << lidar_data.first << " : " << lidar_data.second << std::endl;

    //     motors = obstacleController.calculateMotors(lidar_data.first);
    //     std::cout << "Motor L: " << motors.first << std::endl << "Motor R: " << motors.second << std::endl;
    // }
    

    // return 0;
}

// #include "ldlidar_driver.h"

// uint64_t GetSystemTimeStamp(void) {
//   std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
//     std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
//   auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
//   return ((uint64_t)tmp.count());
// }

// int main() {

//   std::string port_name = "/dev/lidar";
//   uint32_t serial_baudrate = 230400;
//   ldlidar::LDType type_name = ldlidar::LDType::LD_19;

//   ldlidar::LDLidarDriver* node = new ldlidar::LDLidarDriver();

//   node->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp)); 

//   node->EnableFilterAlgorithnmProcess(true);

//   node->Start(type_name, port_name, serial_baudrate, ldlidar::COMM_SERIAL_MODE);

//   if (node->WaitLidarCommConnect(3500)) {
//     LDS_LOG_INFO("ldlidar communication is normal.","");
//   } else {
//     LDS_LOG_ERROR("ldlidar communication is abnormal.","");
//     node->Stop();
//   }

//   ldlidar::Points2D laser_scan_points;
  
//   while (ldlidar::LDLidarDriver::IsOk()) {
//     switch (node->GetLaserScanData(laser_scan_points, 1500)){
//       case ldlidar::LidarStatus::NORMAL: {
//         double lidar_scan_freq = 0;
//         node->GetLidarScanFreq(lidar_scan_freq);


//         LDS_LOG_INFO("speed(Hz):%f,size:%d,stamp_front:%lu, stamp_back:%lu",
//           lidar_scan_freq, laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp);

//         //  output 2d point cloud data
//         for (auto point : laser_scan_points) {
//           LDS_LOG_INFO("stamp:%lu,angle:%f,distance(mm):%d,intensity:%d", 
//             point.stamp, point.angle, point.distance, point.intensity);
//         }
        
//         break;
//       }
//       case ldlidar::LidarStatus::DATA_TIME_OUT: {
//         LDS_LOG_ERROR("ldlidar publish data is time out, please check your lidar device.","");
//         node->Stop();
//         break;
//       }
//       case ldlidar::LidarStatus::DATA_WAIT: {
//         break;
//       }
//       default: {
//         break;
//       }
//     }

//     usleep(1000 * 100);  // sleep 100ms  == 10Hz
//   }

//   node->Stop();
//   // LidarPowerOff();

//   delete node;
//   node = nullptr;

//   return 0;
// }


// int main() {

//     float var1;
//     float var2;

//     while(true) {
//         var1 = actualizar_var1(); // se actualiza la variable 1
//         var2 = actualizar_var2(); // se actualiza la variable 1

//         // Guardar la variable 1 en el archivo senal_con_ruido.txt
//         ...

//         // Guardar la variable 2 en el archivo senal_filtrada.txt
//         ...   
//     }

//     return 0;
// }