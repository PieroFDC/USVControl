add_library(serial_lib serialib/serialib.cpp)
add_library(tiny_xml2 tinyxml2/tinyxml2.cpp)

target_include_directories(serial_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(tiny_xml2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})