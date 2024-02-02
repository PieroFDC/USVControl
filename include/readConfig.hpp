#ifndef __READCONFIG_HPP
#define __READCONFIG_HPP

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "utils.hpp"

struct Config {
    bool Course;
    bool Obstacle;
    bool Collection;
    bool ShowCameraFrames;
};

Config readYAML() {
    YAML::Node node = YAML::LoadFile(CONFIG_PATH);

    Config config;

    config.Course = node["Controller"]["Course"].as<bool>();
    config.Obstacle = node["Controller"]["Obstacle"].as<bool>();
    config.Collection = node["Controller"]["Collection"].as<bool>();
    config.ShowCameraFrames = node["ShowCameraFrames"].as<bool>();

    return config;
}

#endif //__READCONFIG_HPP