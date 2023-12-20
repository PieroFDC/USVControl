cmake_minimum_required(VERSION 3.0)
project(USVControl VERSION 1.0)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(FUZZYLITE_PATH "/home/piero/Documents/Libs/fuzzylite/fuzzylite")


set(ncnn_DIR "/home/piero/Documents/Libs/ncnn/build/install/lib/cmake/ncnn" CACHE PATH "/home/piero/Documents/Libs/ncnn/build/install/lib/cmake/ncnn/ncnnConfig.cmake")
find_package(ncnn REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${ncnn_INCLUDE_DIRS} ${FUZZYLITE_PATH})


add_library(fuzzylite SHARED IMPORTED)
set_target_properties(fuzzylite PROPERTIES IMPORTED_LOCATION "${FUZZYLITE_PATH}/debug/bin/libfuzzylite-debug.so")


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/core/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/dataprocess/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/filter/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/logger/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/networkcom/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/serialcom/
)

file(GLOB  LDLIDAR_DRI_CORE ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/core/*.cpp)
file(GLOB  LDLIDAR_DRI_DATARPC ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/dataprocess/*.cpp)
file(GLOB  LDLIDAR_DRI_FILTER ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/filter/*.cpp)
file(GLOB  LDLIDAR_DRI_LOGGER ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/logger/*.cpp)
file(GLOB  LDLIDAR_DRI_NETWORK ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/networkcom/*.cpp)
file(GLOB  LDLIDAR_DRI_SERIAL ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/serialcom/*.cpp)

add_subdirectory(lib)

file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.hpp" "include/controllers/*.hpp")

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})


target_include_directories(${PROJECT_NAME} PRIVATE include)


add_library(ldlidar_driver_static STATIC 
  ${LDLIDAR_DRI_CORE} 
  ${LDLIDAR_DRI_DATARPC} 
  ${LDLIDAR_DRI_FILTER} 
  ${LDLIDAR_DRI_LOGGER} 
  ${LDLIDAR_DRI_NETWORK} 
  ${LDLIDAR_DRI_SERIAL}
)

set_target_properties (ldlidar_driver_static PROPERTIES OUTPUT_NAME "ldlidar_driver")

target_link_libraries(
    ${PROJECT_NAME} PUBLIC
    ${OpenCV_LIBS}
    fuzzylite
    ncnn
    ldlidar_driver_static
    serial_lib)

# Habilitar OpenMP si estás utilizando -fopenmp en el script original
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

INSTALL(TARGETS ldlidar_driver_static
  ARCHIVE DESTINATION lib/ldlidar_driver
)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/ldlidar_driver/include
  DESTINATION include/ldlidar_driver
)
