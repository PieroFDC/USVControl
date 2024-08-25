#ifndef FUNCTIONUTILS_HPP
#define FUNCTIONUTILS_HPP

#include <iostream>
#include <cmath>

constexpr double DEG_TO_RAD = M_PI / 180;
constexpr double RAD_TO_DEG = 180 / M_PI;

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    lat1 *= DEG_TO_RAD;
    lon1 *= DEG_TO_RAD;
    lat2 *= DEG_TO_RAD;
    lon2 *= DEG_TO_RAD;

    double deltaLon = lon2 - lon1;

    double y = sin(deltaLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(deltaLon);
    double bearing = atan2(y, x) * RAD_TO_DEG;  // Convertir a grados

    bearing = fmod((bearing + 360), 360);

    return bearing;
}

double angleBetweenVectors(double lat1, double lon1, double heading, double lat2, double lon2) {
    double bearing = calculateBearing(lat1, lon1, lat2, lon2);

    double angle = bearing - heading;

    if (angle > 180) {
        angle -= 360;
    } else if (angle < -180) {
        angle += 360;
    }

    return -angle;
}

double correctAngle(double declination, double angle) {
    angle -= declination;

    if (angle > 180) {
        angle -= 360;
    } else if (angle < -180) {
        angle += 360;
    }

    return angle;
}

#endif //FUNCTIONUTILS_HPP
