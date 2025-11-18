cmake_minimum_required(VERSION 3.12)
project(phidget_homer)

add_executable(phidget_homer phidget_homer.cpp)
target_link_libraries(phidget_homer phidget22)
target_compile_features(phidget_homer PRIVATE cxx_std_17)
