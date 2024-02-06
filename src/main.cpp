#include "main.hpp"

struct ControllerOutput {
    bool detection;
    std::pair<float, float> motors;
};

void course_controller_thread(std::queue<ControllerOutput>& resultQueue,
                                    std::mutex& mutex,
                                    std::condition_variable& cv,
                                    std::atomic<float>& EOvalue) {

    CourseController courseController;
    ControllerOutput controller_output = {true, {1500, 1500}};

    while(true) {
        float current_eo_value = EOvalue.load();

        controller_output.motors = courseController.calculateMotors(current_eo_value);

        {
            std::lock_guard<std::mutex> lock(mutex);
            resultQueue.push(controller_output);
        }

        cv.notify_one();
    }
}

void obstacle_controller_thread(std::queue<ControllerOutput>& resultQueue,
                                    std::mutex& mutex,
                                    std::condition_variable& cv) {

    int max_distance_obstacle = 1;
    bool init_lidar = false;
    
    ObstacleController obstacleController;
    ControllerOutput controller_output = {false, {1500, 1500}};

    std::pair<float, float> lidar_data;
    LidarSensor lidar_sensor;


    init_lidar = lidar_sensor.InitializeLidar();
    
    do {
        try {
            lidar_data = lidar_sensor.RunLidar(max_distance_obstacle);

            if(!(lidar_data.first || lidar_data.second)) {
                controller_output.motors = {1500, 1500};
                controller_output.detection = false;
            } else {
                controller_output.motors = obstacleController.calculateMotors(lidar_data.first);
                controller_output.detection = true;
            }

        } catch (const std::exception& e) {
            std::cerr << "Error al leer los datos del lidar." << std::endl;
        }

        {
            std::lock_guard<std::mutex> lock(mutex);
            resultQueue.push(controller_output);
        }

        cv.notify_one();

    } while(init_lidar);
}

void collection_controller_thread(std::queue<ControllerOutput>& resultQueue,
                                    std::mutex& mutex,
                                    std::condition_variable& cv) {

    CollectionController collectionController;
    ControllerOutput controller_output = {false, {1500, 1500}};

    ObjectDetector detector(
        MODEL_PARAM_PATH.c_str(),
        MODEL_BIN_PATH.c_str(),
        352,
        352
    ); 

    cv::VideoCapture cap(0);
    cv::Mat frame;

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    std::pair<float, float> center;
    bool camera_ok = true;

    if (!cap.isOpened()) {
        std::cerr << "Error al abrir la cámara." << std::endl;
        camera_ok = false;
    }

    do {
        try {
            cap >> frame;
            center = detector.detectLoop(frame);

            if(center.first && center.second) {
                controller_output.detection = true;
                controller_output.motors = collectionController.calculateMotors(center);
            } else {
                controller_output.detection = false;
                controller_output.motors = {1500, 1500};
            }
            

        } catch (const std::exception& e) {
            std::cerr << "Error al detectar las imágenes." << std::endl;
        }


        if (cv::waitKey(1) == 'q')
            break;

        {
            std::lock_guard<std::mutex> lock(mutex);
            resultQueue.push(controller_output);
        }

        cv.notify_one();

    } while(camera_ok && !frame.empty());
 }

int main() {
    Config project_config = readYAML();

    std::cout << project_config.Collection << " : " << project_config.Course << " : " << project_config.Obstacle << std::endl;

    std::thread course_thread;
    std::thread obstacle_thread;
    std::thread collection_thread;

    ControllerOutput output_course;
    ControllerOutput output_obstacle;
    ControllerOutput output_collection;

    std::queue<ControllerOutput> result_queue_course;
    std::queue<ControllerOutput> result_queue_obstacle;
    std::queue<ControllerOutput> result_queue_collection;

    std::mutex mutex_course;
    std::mutex mutex_obstacle;
    std::mutex mutex_collection;

    std::condition_variable cv_course;
    std::condition_variable cv_obstacle;
    std::condition_variable cv_collection;

    std::atomic<float> input_eo(0);

    if(project_config.Course) {
        course_thread = std::thread(
            course_controller_thread,
            std::ref(result_queue_course),
            std::ref(mutex_course),
            std::ref(cv_course),
            std::ref(input_eo)
        );
    }

    if(project_config.Obstacle) {
        obstacle_thread = std::thread(
            obstacle_controller_thread,
            std::ref(result_queue_obstacle),
            std::ref(mutex_obstacle),
            std::ref(cv_obstacle)
        );
    }

    if(project_config.Collection) {
        collection_thread = std::thread(
            collection_controller_thread,
            std::ref(result_queue_collection),
            std::ref(mutex_collection),
            std::ref(cv_collection)
        );
    }
    
    bool obstacle_thread_exec = true;
    bool collection_thread_exec = true;

    std::chrono::milliseconds timeout_read(1);

    Filter filter(0.3);

    float filt_value;

    SerialCommunication serialComm;

    SensorDataInput sdatainp;
    SensorDataOutput sdataout;

    sdataout.camera_yaw = 78.0;
    sdataout.nrf = "nrf";
    sdataout.pwml = 1500;
    sdataout.pwmr = 1500;

    serialComm.sendData(sdataout);

    while(true) {
        auto start_time_exec = std::chrono::high_resolution_clock::now(); 

        while(true) {
            try {
                sdatainp = serialComm.receiveData();   
                break;
            } catch (const std::exception& e) {
                std::cerr << "Error receiving data: " << e.what() << std::endl;
                serialComm.sendData(sdataout);
            }
        }

        input_eo.store(sdatainp.yaw);

        if(project_config.Course) {
            std::unique_lock<std::mutex> lock(mutex_course);
            cv_course.wait_for(
                lock,
                timeout_read,
                [&result_queue_course] {
                    return !result_queue_course.empty();
                }
            );

            while(!result_queue_course.empty()) {
                output_course = result_queue_course.front();
                result_queue_course.pop();
            }
        }

        if(project_config.Obstacle) {
            std::unique_lock<std::mutex> lock(mutex_obstacle);
            cv_obstacle.wait_for(
                lock,
                timeout_read,
                [&result_queue_obstacle] {
                    return !result_queue_obstacle.empty();
                }
            );

            while(!result_queue_obstacle.empty()) {
                output_obstacle = result_queue_obstacle.front();
                result_queue_obstacle.pop();
            }
        }

        if(project_config.Collection) {
            std::unique_lock<std::mutex> lock(mutex_collection);
            cv_collection.wait_for(
                lock,
                timeout_read,
                [&result_queue_collection] {
                    return !result_queue_collection.empty();
                }
            );

            while(!result_queue_collection.empty()) {
                output_collection = result_queue_collection.front();
                result_queue_collection.pop();
            }
        }

        // filt_value = filter.filterData(output_course.motors.first);

        // std::cout << "\033[2J\033[1;1H"; // Clear console
        std::cout << "YAW: " << sdatainp.yaw << std::endl;
        // std::cout << output_course.motors.first << " : " << filt_value << std::endl;
        // std::cout << "Resultado leído del programa Course: " << output_course.motors.first << " : " << output_course.motors.second << " : " << output_course.detection << std::endl;
        // std::cout << "Resultado leído del programa Obstacle: " << output_obstacle.motors.first << " : " << output_obstacle.motors.second << " : " << output_obstacle.detection << std::endl;
        // std::cout << "Resultado leído del programa Collection: " << output_collection.motors.first << " : " << output_collection.motors.second << " : " << output_collection.detection << std::endl;

        serialComm.sendData(sdataout);     

        auto end_time_exec = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_exec - start_time_exec);

        std::cout << "duration: " << duration.count() << std::endl;

    }
    std::cout << "Fin del programa ..." << std::endl;
    return 0;
}