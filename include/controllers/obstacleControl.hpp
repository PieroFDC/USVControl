#ifndef OBSTACLECONTROL_HPP
#define OBSTACLECONTROL_HPP

#include "fl/Headers.h"
#include "../utils.hpp"

using namespace fl;

class ObstacleController {
public:
    ObstacleController() {
        engine = std::unique_ptr<Engine>(FllImporter().fromFile(OBSTACLE_CONTROL_PATH_FLL.string()));

        std::string status;
        if (not engine->isReady(&status))
            throw Exception("[engine error] Collection Controller Engine is not ready:\n" + status, FL_AT);

        MA = std::unique_ptr<InputVariable>(engine->getInputVariable("MA"));
        FMI = std::unique_ptr<OutputVariable>(engine->getOutputVariable("FMI"));
        FMD = std::unique_ptr<OutputVariable>(engine->getOutputVariable("FMD"));
    }

    std::pair<double, double> calculateMotors(double MAvalue) {
        std::pair<double, double> motors = {1500.0f, 1500.0f};

        MA->setValue(MAvalue);
        engine->process();

        motors.first = FMI->getValue();
        motors.second = FMD->getValue();

        return motors;
    }

private:
    std::unique_ptr<Engine> engine;
    std::unique_ptr<InputVariable> MA;
    std::unique_ptr<OutputVariable> FMI;
    std::unique_ptr<OutputVariable> FMD;
};

#endif //OBSTACLECONTROL_HPP