#ifndef FUNCTIONUTILS_HPP
#define FUNCTIONUTILS_HPP

#include <cmath>

double angleBetweenVectors(double x1, double y1, double heading, double x2, double y2) {
    double theta = heading;
    theta += 90;

    if(theta > 180) {
        theta -= 360;
    }

    theta = theta * (M_PI / 180);

    double AB[2] = {x2-x1, y2-y1};
    double AD[2] = {cos(theta), sin(theta)};

    double cosBeta = (AD[0] * AB[0] + AD[1] * AB[1]) / (sqrt(AD[0] * AD[0] + AD[1] * AD[1]) * sqrt(AB[0] * AB[0] + AB[1] * AB[1]));
    double beta = acos(cosBeta);
    double cross = AD[0] * AB[1] - AD[1] * AB[0];
    int sign;

    if (cross >= 0) {
        sign = 1;
    } else {
        sign = -1;
    }

    double angle = sign * beta * 180 / M_PI;
    return angle;
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
