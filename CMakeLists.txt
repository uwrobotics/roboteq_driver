cmake_minimum_required(VERSION 2.8.3)
project(roboteq_driver)

set(CMAKE_CXX_STANDARD 14)

# Output Dirs
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)

# Clang Tidy
set(CMAKE_CXX_CLANG_TIDY clang-tidy-9)

#inlcuding header files
include_directories(include)

add_library(${PROJECT_NAME}
            src/canopen_comm.cpp
            src/roboteq_controller.cpp)

target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra -Wpedantic -Werror)

add_executable(roboteq_driver_example_main
               src/example_main.cpp)
               
target_link_libraries(roboteq_driver_example_main 
                      ${PROJECT_NAME})
