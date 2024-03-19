#ifndef COURSECONTROL_HPP
#define COURSECONTROL_HPP

#include "fl/Headers.h"
#include "../utils.hpp"

using namespace fl;

class CourseController {
public:
    CourseController() {
        engine = std::unique_ptr<Engine>(FllImporter().fromFile(COURSE_CONTROL_PATH_FLL.string()));

        std::string status;
        if (not engine->isReady(&status))
            throw Exception("[engine error] Course Controller Engine is not ready:\n" + status, FL_AT);

        EO = std::unique_ptr<InputVariable>(engine->getInputVariable("EO"));
        FMI = std::unique_ptr<OutputVariable>(engine->getOutputVariable("FMI"));
        FMD = std::unique_ptr<OutputVariable>(engine->getOutputVariable("FMD"));
    }

    std::pair<double, double> calculateMotors(double EOvalue) {
        std::pair<double, double> motors = {1500.0f, 1500.0f};

        EO->setValue(EOvalue);
        engine->process();

        motors.first = FMI->getValue();
        motors.second = FMD->getValue();

        return motors;
    }

private:
    std::unique_ptr<Engine> engine;
    std::unique_ptr<InputVariable> EO;
    std::unique_ptr<OutputVariable> FMI;
    std::unique_ptr<OutputVariable> FMD;
};

#endif //COURSECONTROL_HPP