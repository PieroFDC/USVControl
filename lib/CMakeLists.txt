add_library(serial_lib serialib/serialib.cpp)
add_library(filter_fir filter/filt.cpp)

target_include_directories(serial_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})