#ifndef FILTER_HPP
#define FILTER_HPP

class Filter {
private:
    double exponentialSmoothingFactor;       // Factor de suavizado exponencial
    double previousMeasurement = 0;          // Variable privada para almacenar la medición anterior

public:
    explicit Filter(double smoothingFactor) : exponentialSmoothingFactor(smoothingFactor) {}

    double filterData(double measurement) {
        double smoothedValue = (exponentialSmoothingFactor * measurement) + ((1 - exponentialSmoothingFactor) * previousMeasurement);
        previousMeasurement = smoothedValue;
        return smoothedValue;
    }
};

#endif //FILTER_HPP