#ifndef __UTILS_HPP
#define __UTILS_HPP

#include <iostream>
#include <filesystem>

std::filesystem::path PROJECT_PATH = std::filesystem::current_path().parent_path();

std::filesystem::path COLLECTION_CONTROL_PATH_FLL = PROJECT_PATH / "fuzzyControllers" / "CollectionControl.fll";
std::filesystem::path COURSE_CONTROL_PATH_FLL = PROJECT_PATH / "fuzzyControllers" / "CourseControl.fll";
std::filesystem::path OBSTACLE_CONTROL_PATH_FLL = PROJECT_PATH / "fuzzyControllers" / "ObstacleControl.fll";

std::filesystem::path MODEL_BIN_PATH = PROJECT_PATH / "model" / "FastestDet.bin";
std::filesystem::path MODEL_PARAM_PATH = PROJECT_PATH / "model" / "FastestDet.param";

#endif //__UTILS_HPP