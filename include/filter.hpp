#ifndef __FILTER_HPP
#define __FILTER_HPP

class Filter {
private:
    float exponentialSmoothingFactor;       // Factor de suavizado exponencial
    float previousMeasurement = 0;          // Variable privada para almacenar la medición anterior

public:
    Filter(float smoothingFactor) : exponentialSmoothingFactor(smoothingFactor) {}

    float filterData(float measurement) {
        float smoothedValue = (exponentialSmoothingFactor * measurement) + ((1 - exponentialSmoothingFactor) * previousMeasurement);
        previousMeasurement = smoothedValue;
        return smoothedValue;
    }
};

#endif //__FILTER_HPP