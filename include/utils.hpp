#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <filesystem>

std::filesystem::path PROJECT_PATH = std::filesystem::current_path().parent_path();

std::filesystem::path COLLECTION_CONTROL_PATH_FLL = PROJECT_PATH / "fuzzyControllers" / "CollectionControl.fll";
std::filesystem::path COURSE_CONTROL_PATH_FLL = PROJECT_PATH / "fuzzyControllers" / "CourseControl.fll";
std::filesystem::path OBSTACLE_CONTROL_PATH_FLL = PROJECT_PATH / "fuzzyControllers" / "ObstacleControl.fll";

std::filesystem::path MODEL_BIN_PATH = PROJECT_PATH / "model" / "BottleDet.bin";
std::filesystem::path MODEL_PARAM_PATH = PROJECT_PATH / "model" / "BottleDet.param";

std::filesystem::path CONFIG_PATH = PROJECT_PATH / "config.yaml";
std::filesystem::path WAYPOINTS_PATH = PROJECT_PATH / "waypoints.gpx";

#endif //UTILS_HPP