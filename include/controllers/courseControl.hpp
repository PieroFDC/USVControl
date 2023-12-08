#ifndef __COURSECONTROL_HPP
#define __COURSECONTROL_HPP

#include <vector>
#include "fl/Headers.h"

#include "../utils.hpp"

using namespace fl;

class CourseController {
public:
    CourseController() {
        // Constructor: Se llama una vez al inicializar la clase
        engine = std::unique_ptr<Engine>(FllImporter().fromFile(COURSE_CONTROL_PATH_FLL.string()));

        std::string status;
        if (not engine->isReady(&status))
            throw Exception("[engine error] Course Controller Engine is not ready:\n" + status, FL_AT);

        EO = std::unique_ptr<InputVariable>(engine->getInputVariable("EO"));
        FMI = std::unique_ptr<OutputVariable>(engine->getOutputVariable("FMI"));
        FMD = std::unique_ptr<OutputVariable>(engine->getOutputVariable("FMD"));
    }

    std::pair<float, float> calculateMotors(float EOvalue) {
        // Método para realizar la inferencia
        std::pair<float, float> motors = {1500.0f, 1500.0f};

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

#endif //__COURSECONTROL_HPP