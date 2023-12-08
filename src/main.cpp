#include "main.hpp"
#include <chrono>
#include <thread>
#include <random>

// std::vector<double> convertAngles(const std::vector<double>& angles) {
//     std::vector<double> convertedAngles;
//     for (double angle : angles) {
//         if (angle >= 0 && angle <= 180) {
//             convertedAngles.push_back(angle);
//         } else if (angle > 180 && angle <= 360) {
//             convertedAngles.push_back(angle - 360);
//         }
//     }
//     return convertedAngles;
// }

// std::pair<double, double> getMinDistanceAngle(const std::vector<double>& angles, const std::vector<double>& distances) {
//     double minDistance = std::numeric_limits<double>::infinity();
//     double minAngle = 0.0;  // Inicializado con un valor arbitrario; asegúrate de que tiene sentido en tu contexto

//     for (size_t i = 0; i < distances.size(); ++i) {
//         if (distances[i] >= 0.05 && distances[i] < minDistance) {
//             minDistance = distances[i];
//             minAngle = angles[i];
//         }
//     }

//     return std::make_pair(minAngle, minDistance);
// }

int main() {
    // float inp; //
    // std::pair<float, float> motors;

    // CollectionController collectionController;
    // CourseController courseController;
    // ObstacleController obstacleController;

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


///////////////////// NCNN
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
    // if (!cap.isOpened()) {
    //     printf("Error al abrir la cámara.\n");
    //     return -1;
    // }

//     while (true) {
//         // Capturar frame de la cámara
//         cv::Mat frame;
//         cap >> frame;

//         if (frame.empty()) {
//             printf("Error al capturar frame de la cámara.\n");
//             break;
//         }

//         center = detector.detectLoop(frame);

//         std::cout << center.first << std::endl << center.second << std::endl;

//         // motors = collectionController.calculateMotors(center);
//         // std::cout << "Motor L: " << motors.first << std::endl << "Motor R: " << motors.second << std::endl;

//         // Romper el bucle si se presiona la tecla 'q'
//         if (cv::waitKey(1) == 'q')
//             break;
//     }

//     cap.release();
//     cv::destroyAllWindows();


    // Inicializar el generador de números aleatorios
    std::random_device rd;
    std::mt19937 gen(rd());

    // Definir el rango
    std::uniform_int_distribution<int> distribution(1100, 1900);

    // Generar dos números aleatorios
    int numero1;
    int numero2;

    SerialCommunication serialComm("/dev/ttyACM0", 2000000);

    SensorData sdata;

    // bucle principal
    while(true) {
        numero1 = distribution(gen);
        numero2 = distribution(gen);

        std::string data_to_send = "<" + std::to_string(numero1) + "," + std::to_string(numero2) + ",78.0,nrf>";

        // // Envía datos al Arduino
        serialComm.sendData(data_to_send);
        std::cout << "Datos enviados: " << data_to_send << std::endl;

        // Ejemplo de uso: Recibir datos
        sdata = serialComm.receiveData();

        // Muestra los datos leídos
        // std::cout << "Datos leídos: " << receivedData << std::endl;
        std::cout << "Lat: " << sdata.lat << std::endl;
        std::cout << "Lon: " << sdata.lon << std::endl;
        std::cout << "Speed: " << sdata.speed << std::endl;
        std::cout << "Yaw: " << sdata.yaw << std::endl;
        std::cout << "NRF: " << sdata.nrf << std::endl;
        std::cout << "Sonic: " << sdata.sonic << std::endl;
        std::cout << "Volt: " << sdata.volt << std::endl;
        std::cout << "PWML: " << sdata.pwml << std::endl;
        std::cout << "PWMR: " << sdata.pwmr << std::endl;
    }

    return 0;
}