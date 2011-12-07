PROJECT(Simox)

cmake_minimum_required(VERSION 2.6)
MESSAGE(STATUS "******************** Configuring Simox ************************")
set (SIMOX_BUILD_DIRECTORY ${CMAKE_BINARY_DIR} CACHE STRING "Simox build directory")

INCLUDE (config.cmake)


MESSAGE(STATUS "\n** SETTING Simox target directory to: ${SIMOX_BUILD_DIRECTORY}")
add_subdirectory(VirtualRobot)
add_subdirectory(MotionPlanning)
add_subdirectory(GraspPlanning)


