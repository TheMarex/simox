PROJECT(Simox)

cmake_minimum_required(VERSION 2.6)
MESSAGE(STATUS "******************** Configuring Simox ************************")
set (Simox_BUILD_DIRECTORY ${CMAKE_BINARY_DIR} CACHE STRING "Simox build directory")

INCLUDE (config.cmake)


MESSAGE(STATUS "\n** SETTING Simox target directory to: ${Simox_BUILD_DIRECTORY}")
add_subdirectory(VirtualRobot)
add_subdirectory(MotionPlanning)
add_subdirectory(GraspPlanning)
add_subdirectory(doc)


#######################################################################
# Setup for installation
#######################################################################

############################################
MESSAGE (STATUS "\n ***** Simox: Generating CMake files for build and install setup")

set(Simox_MAJOR_VERSION 2)
set(Simox_MINOR_VERSION 1)
set(Simox_PATCH_VERSION 3)
set(Simox_VERSION
    ${Simox_MAJOR_VERSION}.${Simox_MINOR_VERSION}.${Simox_PATCH_VERSION})

MESSAGE (STATUS " ** Simox version: ${Simox_VERSION}")

set(Simox_LIBRARIES VirtualRobot Saba GraspStudio)
set(Simox_EXECUTABLES "")

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
SET (VirtualRobot_CONFIG_INBUILD "${Simox_BUILD_DIRECTORY}/VirtualRobotConfig.cmake")
SET (VirtualRobot_CONFIG_INSTALL_LOCAL "${Simox_BUILD_DIRECTORY}/InstallFiles/VirtualRobotConfig.cmake")
#MESSAGE ("VirtualRobot_CONFIG_INBUILD = ${VirtualRobot_CONFIG_INBUILD}")
#MESSAGE ("VirtualRobot_CONFIG_INSTALL_LOCAL = ${VirtualRobot_CONFIG_INSTALL_LOCAL}")
include ("${VirtualRobot_CONFIG_INBUILD}")

# Create an SimoxConfig.cmake file for the use from the build tree
SET(Simox_LIBRARIES_ALL "${VirtualRobot_LIBRARIES}" "Saba" "GraspStudio")
SET(Simox_BASE_DIR "${PROJECT_SOURCE_DIR}")
SET(Simox_INCLUDE_DIRS "${VirtualRobot_INCLUDE_DIRS}" "${GRASP_STUDIO_INCLUDE_DIRS}" "${SABA_INCLUDE_DIRS}")
SET(Simox_LIB_DIRS "${VirtualRobot_LIBRARY_DIRS}" "${GRASP_STUDIO_LIBRARY_DIRS}" "${SABA_LIBRARY_DIRS}")
SET(Simox_LIB_FLAGS "${VirtualRobot_COMPILE_FLAGS} ${GRASP_STUDIO_COMPILE_FLAGS} ${SABA_COMPILE_FLAGS}")
SET(Simox_CMAKE_DIR "${Simox_BUILD_DIRECTORY}/share/Simox/cmake")
SET(VirtualRobot_CMAKE_DIR "${Simox_BUILD_DIRECTORY}/share/VirtualRobot/cmake")
SET(Simox_LIBRARY_CMAKE_INCLUDE "${VirtualRobot_CMAKE_INCLUDE}")
SET(Simox_DATA_PATH "${VirtualRobot_DATA_PATH}")

# the SimoxConfig.cmake will be used by external porjects to setup Simox via the FindPackage method.
# So it must be located in the Simox_DIR directory (build-tree -> Simox_BUILD_DIRECTORY)
configure_file(
    ${PROJECT_SOURCE_DIR}/CMakeModules/SimoxConfig.cmake.in
    "${Simox_BUILD_DIRECTORY}/SimoxConfig.cmake"
    @ONLY)

configure_file(
    ${PROJECT_SOURCE_DIR}/CMakeModules/SimoxConfigVersion.cmake.in
    "${Simox_BUILD_DIRECTORY}/SimoxConfigVersion.cmake"
    @ONLY)

configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/SimoxExternalLibrarySetup.cmake"
    "${Simox_BUILD_DIRECTORY}/share/Simox/cmake/SimoxExternalLibrarySetup.cmake" COPYONLY)
configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/SimoxMacros.cmake"
    "${Simox_BUILD_DIRECTORY}/share/Simox/cmake/SimoxMacros.cmake" COPYONLY)
configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/FindSimox.cmake"
    "${Simox_BUILD_DIRECTORY}/share/Simox/cmake/FindSimox.cmake" COPYONLY)
 
# Create an SimoxConfig.cmake file for the use from the install tree
# and install it

# import Virtual-Robot configs (install)
# do not include files from install tree here (the files are not installed yet. So include them manually from inbuild)
SET (VirtualRobot_DISABLE_DEPENDENCIES_INCLUDE TRUE)
include ("${VirtualRobot_CONFIG_INSTALL_LOCAL}")
include(${PROJECT_SOURCE_DIR}/VirtualRobot/CMakeModules/VirtualRobotExternalLibrarySetup.cmake)
include(${PROJECT_SOURCE_DIR}/VirtualRobot/CMakeModules/VirtualRobotMacros.cmake)

SET(Simox_INCLUDE_DIRS "${VirtualRobot_INCLUDE_DIRS}" "${GRASP_STUDIO_INCLUDE_DIRS}" "${SABA_INCLUDE_DIRS}")
SET(Simox_LIB_DIRS "${VirtualRobot_LIBRARY_DIRS}" "${GRASP_STUDIO_LIBRARY_DIRS}" "${SABA_LIBRARY_DIRS}")
SET(Simox_LIB_FLAGS "${VirtualRobot_COMPILE_FLAGS} ${GRASP_STUDIO_COMPILE_FLAGS} ${SABA_COMPILE_FLAGS}")
SET(Simox_BASE_DIR "${CMAKE_INSTALL_PREFIX}/include")
SET(Simox_CMAKE_DIR "${CMAKE_INSTALL_PREFIX}/share/Simox/cmake")
SET(VirtualRobot_CMAKE_DIR "${CMAKE_INSTALL_PREFIX}/share/VirtualRobot/cmake")
SET(Simox_DATA_PATH "${CMAKE_INSTALL_PREFIX}/data")
configure_file(
    ${PROJECT_SOURCE_DIR}/CMakeModules/SimoxConfig.cmake.in
    "${Simox_BUILD_DIRECTORY}/InstallFiles/SimoxConfig.cmake"
    @ONLY)
configure_file(
    ${PROJECT_SOURCE_DIR}/CMakeModules/SimoxConfigVersion.cmake.in
    "${Simox_BUILD_DIRECTORY}/InstallFiles/SimoxConfigVersion.cmake"
    @ONLY)
# The SimoxConfig.cmake will be used by external porjects to setup Simox via the FindPackage method.
# So it must be located in the Simox_DIR directory (install-tree -> CMAKE_INSTALL_PREFIX)
install(FILES
    "${Simox_BUILD_DIRECTORY}/InstallFiles/SimoxConfig.cmake"
    "${Simox_BUILD_DIRECTORY}/InstallFiles/SimoxConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
install(FILES
    "${PROJECT_SOURCE_DIR}/CMakeModules/SimoxExternalLibrarySetup.cmake"
    "${PROJECT_SOURCE_DIR}/CMakeModules/SimoxMacros.cmake"
    "${PROJECT_SOURCE_DIR}/CMakeModules/FindSimox.cmake"
    DESTINATION "${Simox_CMAKE_DIR}")
install(FILES
    "${PROJECT_SOURCE_DIR}/CMakeModules/SimoxExternalLibrarySetup.cmake"
    "${PROJECT_SOURCE_DIR}/CMakeModules/SimoxMacros.cmake"
    "${PROJECT_SOURCE_DIR}/CMakeModules/FindSimox.cmake"
    DESTINATION "${Simox_CMAKE_DIR}")


#
#INSTALL(TARGETS Simox 
#        EXPORT SimoxLibraryDepends
#        RUNTIME DESTINATION ${Simox_INSTALL_BIN_DIR}
#        LIBRARY DESTINATION ${Simox_INSTALL_LIB_DIR}
#        ARCHIVE DESTINATION ${Simox_INSTALL_LIB_DIR}
#        )




