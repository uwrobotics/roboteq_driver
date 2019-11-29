cmake_minimum_required(VERSION 2.8.3)
project(roboteq_driver)

set(CMAKE_CXX_STANDARD 14)

#inlcuding header files
include_directories(include)

#manually adding sources
file(GLOB SOURCES "src/*.cpp")

add_executable(roboteq_driver ${SOURCES})
