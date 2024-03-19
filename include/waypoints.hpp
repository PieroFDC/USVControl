#ifndef WAYPOINTS_HPP
#define WAYPOINTS_HPP

#include <vector>
#include <iostream>
#include <cmath>
#include "utils.hpp"
#include "tinyxml2/tinyxml2.hpp"

#define EARTH_RADIUS 6371000

class Waypoints {
public:
    Waypoints() {
        waypoints = readWaypoints(WAYPOINTS_PATH.c_str());
        numWaypoints = static_cast<int>(waypoints.size());

        std::cout << "En total hay " << numWaypoints << " waypoints" << std::endl;
        for (auto& p : waypoints) {
            std::cout << "(" << p.first << ", " << p.second << ")" << std::endl;
        }
    }

    void setUSVCoordinates(double lat, double lon) {
        double distance = distanceBetweenGPS(lat, lon, waypoints[waipointStatus].first, waypoints[waipointStatus].second);

        if(distance < 5) {
            if(waipointStatus < numWaypoints){
                waipointStatus++;
            } else {
                waipointStatus = -1;
            }
        }
    }

    [[nodiscard]] int getWaypointStatus() const { return waipointStatus; }

    std::pair<double, double> getActualWaypoint() {
        if(waipointStatus != -1) {
            return {waypoints[waipointStatus].first, waypoints[waipointStatus].second};
        } else {
            return {};
        }
    }

    void setReturnHome() {
        waipointStatus = numWaypoints;
    }

private:
    std::vector<std::pair<double, double>> waypoints;
    int numWaypoints;
    int waipointStatus = 0;


    static std::vector<std::pair<double, double>> readWaypoints(const char* filename) {
        tinyxml2::XMLDocument doc;

        if (doc.LoadFile(filename) != tinyxml2::XML_SUCCESS) {
            std::cerr << "Error opening file " << filename << std::endl;
            return {};
        }

        tinyxml2::XMLElement* root = doc.RootElement();

        if (root == nullptr || strcmp(root->Name(), "gpx") != 0) {
            std::cerr << "File " << filename << " does not have the correct format" << std::endl;
            return {};
        }

        std::vector<std::pair<double, double>> waypoints;

        for (tinyxml2::XMLElement* wpt = root->FirstChildElement("wpt"); wpt != nullptr; wpt = wpt->NextSiblingElement("wpt")) {
            double lat = wpt->DoubleAttribute("lat");
            double lon = wpt->DoubleAttribute("lon");
            waypoints.emplace_back(lat, lon);
        }

        waypoints.push_back(waypoints[0]);

        return waypoints;
    }

    static double toRadians(double degrees) {
        return degrees * M_PI / 180;
    }

    static double distanceBetweenGPS(double lat1, double lon1, double lat2, double lon2) {
        // Convierte las coordenadas a radianes
        lat1 = toRadians(lat1);
        lon1 = toRadians(lon1);
        lat2 = toRadians(lat2);
        lon2 = toRadians(lon2);

        // Aplica la fórmula del haversine
        double h = pow(sin((lat2 - lat1) / 2), 2) + cos(lat1) * cos(lat2) * pow(sin((lon2 - lon1) / 2), 2);
        double d = 2 * EARTH_RADIUS * asin(sqrt(h));
        return d;
    }
};

#endif //WAYPOINTS_HPP