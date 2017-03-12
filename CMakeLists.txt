cmake_minimum_required (VERSION 3.5)

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

project(ExtendedKF LANGUAGES C CXX)

add_definitions(-std=c++0x)

if (MSVC)
    add_definitions("/MP -D_SCL_SECURE_NO_WARNINGS /W4 /WX /wd4005 /wd4127 /wd4481 /wd4482 /wd4505 /wd4512 /wd4714 /wd4800 /wd4189")
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

file(GLOB sources src/*.cpp src/*.h)

add_executable(ExtendedKF ${sources})
