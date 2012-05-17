PROJECT(Simox)

cmake_minimum_required(VERSION 2.6)
MESSAGE(STATUS "******************** Configuring Simox ************************")
set (SIMOX_BUILD_DIRECTORY ${CMAKE_BINARY_DIR} CACHE STRING "Simox build directory")

INCLUDE (config.cmake)


MESSAGE(STATUS "\n** SETTING Simox target directory to: ${SIMOX_BUILD_DIRECTORY}")
add_subdirectory(VirtualRobot)
add_subdirectory(MotionPlanning)
add_subdirectory(GraspPlanning)
add_subdirectory(doc)


#######################################################################
# Setup for installation
#######################################################################

############################################
MESSAGE(STATUS "Simox: Generating CMake files")

set(SIMOX_MAJOR_VERSION 2)
set(SIMOX_MINOR_VERSION 1)
set(SIMOX_PATCH_VERSION 2)
set(SIMOX_VERSION
    ${SIMOX_MAJOR_VERSION}.${SIMOX_MINOR_VERSION}.${SIMOX_PATCH_VERSION})

set(SIMOX_LIBRARIES VirtualRobot Saba GraspStudio)
set(SIMOX_EXECUTABLES "")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
EXPORT(PACKAGE Simox)

# Install the export set for use with the install-tree
#install(
#    EXPORT SimoxLibraryDepends
#    DESTINATION "${CMAKE_INSTALL_PREFIX}/share/Simox/cmake")


#get_property(VirtualRobot_external_INCLUDE_DIRS TARGET VirtualRobot PROPERTY EXTERNAL_INCLUDE_DIRS)
#message(STATUS "VirtualRobot_external_INCLUDE_DIRS: ${VirtualRobot_external_INCLUDE_DIRS}")

# import Virtual-Robot configs (build)
SET (VIRTUAL_ROBOT_CONFIG_INBUILD "${SIMOX_BUILD_DIRECTORY}/share/VirtualRobot/cmake/VirtualRobotConfig.cmake")
SET (VIRTUAL_ROBOT_CONFIG_INSTALL_LOCAL "${SIMOX_BUILD_DIRECTORY}/InstallFiles/VirtualRobotConfig.cmake")
#MESSAGE ("VIRTUAL_ROBOT_CONFIG_INBUILD = ${VIRTUAL_ROBOT_CONFIG_INBUILD}")
#MESSAGE ("VIRTUAL_ROBOT_CONFIG_INSTALL_LOCAL = ${VIRTUAL_ROBOT_CONFIG_INSTALL_LOCAL}")
include ("${VIRTUAL_ROBOT_CONFIG_INBUILD}")

# Create an SimoxConfig.cmake file for the use from the build tree
SET(SIMOX_LIBRARIES_ALL "${VIRTUAL_ROBOT_LIBRARIES}" "Saba" "GraspStudio")
SET(SIMOX_BASE_DIR "${PROJECT_SOURCE_DIR}")
SET(SIMOX_INCLUDE_DIRS "${VIRTUAL_ROBOT_INCLUDE_DIRS}" "${GRASP_STUDIO_INCLUDE_DIRS}" "${SABA_INCLUDE_DIRS}")
SET(SIMOX_LIB_DIRS "${VIRTUAL_ROBOT_LIBRARY_DIRS}" "${GRASP_STUDIO_LIBRARY_DIRS}" "${SABA_LIBRARY_DIRS}")
SET(SIMOX_LIB_FLAGS "${VIRTUAL_ROBOT_COMPILE_FLAGS} ${GRASP_STUDIO_COMPILE_FLAGS} ${SABA_COMPILE_FLAGS}")
SET(SIMOX_CMAKE_DIR "${SIMOX_BUILD_DIRECTORY}/share/Simox/cmake")
SET(VIRTUAL_ROBOT_CMAKE_DIR "${SIMOX_BUILD_DIRECTORY}/share/VirtualRobot/cmake")
SET(SIMOX_LIBRARY_CMAKE_INCLUDE "${VIRTUAL_ROBOT_CMAKE_INCLUDE}")
SET(SIMOX_DATA_PATH "${VIRTUAL_ROBOT_DATA_PATH}")

# the SimoxConfig.cmake will be used by external porjects to setup Simox via the FindPackage method.
# So it must be located in the Simox_DIR directory (build-tree -> SIMOX_BUILD_DIRECTORY)
configure_file(
    ${PROJECT_SOURCE_DIR}/CMakeModules/SimoxConfig.cmake.in
    "${SIMOX_BUILD_DIRECTORY}/SimoxConfig.cmake"
    @ONLY)

configure_file(
    ${PROJECT_SOURCE_DIR}/CMakeModules/SimoxConfigVersion.cmake.in
    "${SIMOX_BUILD_DIRECTORY}/SimoxConfigVersion.cmake"
    @ONLY)

configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/SimoxExternalLibrarySetup.cmake"
    "${SIMOX_BUILD_DIRECTORY}/share/Simox/cmake/SimoxExternalLibrarySetup.cmake" COPYONLY)
configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/SimoxMacros.cmake"
    "${SIMOX_BUILD_DIRECTORY}/share/Simox/cmake/SimoxMacros.cmake" COPYONLY)
configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/FindSimox.cmake"
    "${SIMOX_BUILD_DIRECTORY}/share/Simox/cmake/FindSimox.cmake" COPYONLY)
 
# Create an SimoxConfig.cmake file for the use from the install tree
# and install it
# import Virtual-Robot configs (install)
include ("${VIRTUAL_ROBOT_CONFIG_INSTALL_LOCAL}")
SET(SIMOX_INCLUDE_DIRS "${VIRTUAL_ROBOT_INCLUDE_DIRS}" "${GRASP_STUDIO_INCLUDE_DIRS}" "${SABA_INCLUDE_DIRS}")
SET(SIMOX_LIB_DIRS "${VIRTUAL_ROBOT_LIBRARY_DIRS}" "${GRASP_STUDIO_LIBRARY_DIRS}" "${SABA_LIBRARY_DIRS}")
SET(SIMOX_LIB_FLAGS "${VIRTUAL_ROBOT_COMPILE_FLAGS} ${GRASP_STUDIO_COMPILE_FLAGS} ${SABA_COMPILE_FLAGS}")
SET(SIMOX_BASE_DIR "${CMAKE_INSTALL_PREFIX}/include")
SET(SIMOX_CMAKE_DIR "${CMAKE_INSTALL_PREFIX}/share/Simox/cmake")
SET(VIRTUAL_ROBOT_CMAKE_DIR "${CMAKE_INSTALL_PREFIX}/share/VirtualRobot/cmake")
SET(SIMOX_DATA_PATH "${CMAKE_INSTALL_PREFIX}/data")
configure_file(
    ${PROJECT_SOURCE_DIR}/CMakeModules/SimoxConfig.cmake.in
    "${SIMOX_BUILD_DIRECTORY}/InstallFiles/SimoxConfig.cmake"
    @ONLY)
configure_file(
    ${PROJECT_SOURCE_DIR}/CMakeModules/SimoxConfigVersion.cmake.in
    "${SIMOX_BUILD_DIRECTORY}/InstallFiles/SimoxConfigVersion.cmake"
    @ONLY)
# The SimoxConfig.cmake will be used by external porjects to setup Simox via the FindPackage method.
# So it must be located in the Simox_DIR directory (install-tree -> CMAKE_INSTALL_PREFIX)
install(FILES
    "${SIMOX_BUILD_DIRECTORY}/InstallFiles/SimoxConfig.cmake"
    "${SIMOX_BUILD_DIRECTORY}/InstallFiles/SimoxConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
install(FILES
    "${PROJECT_SOURCE_DIR}/CMakeModules/SimoxExternalLibrarySetup.cmake"
    "${PROJECT_SOURCE_DIR}/CMakeModules/SimoxMacros.cmake"
    "${PROJECT_SOURCE_DIR}/CMakeModules/FindSimox.cmake"
    DESTINATION "${SIMOX_CMAKE_DIR}")
install(FILES
    "${PROJECT_SOURCE_DIR}/CMakeModules/SimoxExternalLibrarySetup.cmake"
    "${PROJECT_SOURCE_DIR}/CMakeModules/SimoxMacros.cmake"
    "${PROJECT_SOURCE_DIR}/CMakeModules/FindSimox.cmake"
    DESTINATION "${SIMOX_CMAKE_DIR}")


#
#INSTALL(TARGETS Simox 
#        EXPORT SimoxLibraryDepends
#        RUNTIME DESTINATION ${SIMOX_INSTALL_BIN_DIR}
#        LIBRARY DESTINATION ${SIMOX_INSTALL_LIB_DIR}
#        ARCHIVE DESTINATION ${SIMOX_INSTALL_LIB_DIR}
#        )




