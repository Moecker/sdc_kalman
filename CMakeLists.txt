cmake_minimum_required (VERSION 3.5)

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

project(ExtendedKF LANGUAGES C CXX)

add_definitions(-std=c++0x)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

file(GLOB sources src/*.cpp src/*.h)

add_executable(ExtendedKF ${sources})
