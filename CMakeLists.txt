cmake_minimum_required(VERSION 3.0)
project(USVControl VERSION 1.0)

# Buscar paquetes necesarios
find_package(OpenCV REQUIRED)

# set c++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)

set(FUZZYLITE_PATH "/home/piero/Documents/Libs/fuzzylite/fuzzylite")

# Agregar la integración de ncnn
set(ncnn_DIR "/home/piero/Documents/Libs/ncnn/build/install/lib/cmake/ncnn" CACHE PATH "/home/piero/Documents/Libs/ncnn/build/install/lib/cmake/ncnn/ncnnConfig.cmake")
find_package(ncnn REQUIRED)

# Agrega el directorio de FuzzyLite a la lista de directorios de inclusión
include_directories(${FUZZYLITE_PATH})

# Incluir NCNN
include_directories(include/ncnn ${OpenCV_INCLUDE_DIRS} ${ncnn_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})



# Agrega la biblioteca FuzzyLite a tu proyecto
add_library(fuzzylite SHARED IMPORTED)
set_target_properties(fuzzylite PROPERTIES IMPORTED_LOCATION "${FUZZYLITE_PATH}/debug/bin/libfuzzylite-debug.so")

file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.hpp" "include/controllers/*.hpp")

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(
    ${PROJECT_NAME} PUBLIC
    fuzzylite
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ncnn)

# Habilitar OpenMP si estás utilizando -fopenmp en el script original
find_package(OpenMP)

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

