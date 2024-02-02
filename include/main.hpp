#ifndef __MAIN_HPP
#define __MAIN_HPP

#include <chrono>
#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <random>

#include "controllers/collectionControl.hpp"
#include "controllers/courseControl.hpp"
#include "controllers/obstacleControl.hpp"

#include "utils.hpp"
#include "ncnnModel.hpp"
#include "serialCom.hpp"
#include "lidarSensor.hpp"
#include "filter.hpp"
#include "readConfig.hpp"

#include "test.hpp" //

#endif //__MAIN_HPP