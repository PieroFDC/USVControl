#ifndef READCONFIG_HPP
#define READCONFIG_HPP

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "utils.hpp"

struct Config {
    bool Course;
    bool Obstacle;
    bool Collection;
    bool ShowCameraFrames;
    bool ARM;
    float Latitude;
    float Longitude;
};

Config readYAML() {
    YAML::Node node = YAML::LoadFile(CONFIG_PATH);

    Config config{true, true, true, true, false, 0, 0};

    config.Course = node["Controller"]["Course"].as<bool>();
    config.Obstacle = node["Controller"]["Obstacle"].as<bool>();
    config.Collection = node["Controller"]["Collection"].as<bool>();
    config.ShowCameraFrames = node["ShowCameraFrames"].as<bool>();
    config.ARM = node["ARM"].as<bool>();
    config.Latitude = node["GPSData"]["Latitude"].as<float>();
    config.Longitude = node["GPSData"]["Longitude"].as<float>();

    return config;
}

#endif //READCONFIG_HPP