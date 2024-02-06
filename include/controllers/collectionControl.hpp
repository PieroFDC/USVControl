#ifndef __COLLECTIONCONTROL_HPP
#define __COLLECTIONCONTROL_HPP

#include "fl/Headers.h"
#include "../utils.hpp"

using namespace fl;

class CollectionController {
public:
    CollectionController() {
        engine = std::unique_ptr<Engine>(FllImporter().fromFile(COLLECTION_CONTROL_PATH_FLL.string()));

        std::string status;
        if (not engine->isReady(&status))
            throw Exception("[engine error] Collection Controller Engine is not ready:\n" + status, FL_AT);

        OFFSET = std::unique_ptr<InputVariable>(engine->getInputVariable("OFFSET"));
        FMI = std::unique_ptr<OutputVariable>(engine->getOutputVariable("FMI"));
        FMD = std::unique_ptr<OutputVariable>(engine->getOutputVariable("FMD"));
    }

    std::pair<float, float> calculateMotors(std::pair<float, float> OFFSETvalue) {
        // Método para realizar la inferencia
        std::pair<float, float> motors = {1500.0f, 1500.0f};

        OFFSET->setValue(OFFSETvalue.first);
        engine->process();

        motors.first = FMI->getValue();
        motors.second = FMD->getValue();

        return motors;
    }

private:
    std::unique_ptr<Engine> engine;
    std::unique_ptr<InputVariable> OFFSET;
    std::unique_ptr<OutputVariable> FMI;
    std::unique_ptr<OutputVariable> FMD;
};

#endif //__COLLECTIONCONTROL_HPP