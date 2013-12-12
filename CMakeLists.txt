PROJECT(Simox)

cmake_minimum_required(VERSION 2.6)
if(NOT ("${CMAKE_VERSION}" VERSION_LESS 2.8.12))
	cmake_policy(SET CMP0022 OLD) # avoid INTERFACE_LINK_LIBRARIES warninss
ENDIF()

MESSAGE(STATUS "******************** Configuring Simox ************************")
set (Simox_BUILD_DIRECTORY ${CMAKE_BINARY_DIR} CACHE STRING "Simox build directory")
#set (Simox_BUILD_VirtualRobot TRUE CACHE BOOL "Build Simox VirtualRobot")
set (Simox_BUILD_Saba TRUE CACHE BOOL "Build Motion Planning library")
set (Simox_BUILD_GraspStudio TRUE CACHE BOOL "Build Grasp Planning library")
set (Simox_BUILD_SimDynamics FALSE CACHE BOOL "Build Dynamic Simulation")

INCLUDE (config.cmake)


MESSAGE(STATUS "\n** SETTING Simox target directory to: ${Simox_BUILD_DIRECTORY}")
add_subdirectory(VirtualRobot)
if (Simox_BUILD_Saba)
    add_subdirectory(MotionPlanning)
endif()
if (Simox_BUILD_GraspStudio)
    add_subdirectory(GraspPlanning)
endif()
if (Simox_BUILD_SimDynamics)
    add_subdirectory(SimDynamics)
endif()
add_subdirectory(doc)


#######################################################################
# Setup for installation
#######################################################################

############################################
MESSAGE (STATUS "\n ***** Simox: Generating CMake files for build and install setup")

set(Simox_LIBRARIES VirtualRobot Saba GraspStudio)
if (Simox_BUILD_SimDynamics)
	set(Simox_LIBRARIES "Simox_LIBRARIES" "SimDynamics")
endif()
set(Simox_EXECUTABLES "")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
EXPORT(PACKAGE Simox)

# import Virtual-Robot configs (build)
SET (VirtualRobot_CONFIG_INBUILD "${Simox_BUILD_DIRECTORY}/share/VirtualRobot/cmake/VirtualRobotConfig.cmake")
SET (VirtualRobot_CONFIG_INSTALL_LOCAL "${Simox_BUILD_DIRECTORY}/InstallFiles/VirtualRobotConfig.cmake")
#MESSAGE ("VirtualRobot_CONFIG_INBUILD = ${VirtualRobot_CONFIG_INBUILD}")
#MESSAGE ("VirtualRobot_CONFIG_INSTALL_LOCAL = ${VirtualRobot_CONFIG_INSTALL_LOCAL}")
include ("${VirtualRobot_CONFIG_INBUILD}")

# Create an SimoxConfig.cmake file for the use from the build tree
SET(Simox_LIBRARIES_ALL "${VirtualRobot_LIBRARIES}" "Saba" "GraspStudio")
if (Simox_BUILD_SimDynamics)
	SET(Simox_LIBRARIES_ALL "${Simox_LIBRARIES_ALL}" "SimDynamics" "${SimDynamics_EXTERNAL_LIBRARIES}")
	#MESSAGE ("Simox_LIBRARIES_ALL: ${Simox_LIBRARIES_ALL}")
	#MESSAGE ("SimDynamics_PHYSICS_LIBRARIES: ${SimDynamics_PHYSICS_LIBRARIES}")
	#MESSAGE ("SABA_INCLUDE_DIRS: ${SABA_INCLUDE_DIRS}")

endif()
SET(Simox_BASE_DIR "${PROJECT_SOURCE_DIR}")
SET(Simox_INCLUDE_DIRS "${VirtualRobot_INCLUDE_DIRS}" "${GRASP_STUDIO_INCLUDE_DIRS}" "${SABA_INCLUDE_DIRS}" "${SIMDYNAMICS_INCLUDE_DIRS}")
	#MESSAGE ("VirtualRobot_INCLUDE_DIRS: ${VirtualRobot_INCLUDE_DIRS}")
	#MESSAGE ("Simox_INCLUDE_DIRS: ${Simox_INCLUDE_DIRS}")
	#MESSAGE ("Simox_LIBRARIES_ALL: ${Simox_LIBRARIES_ALL}")
SET(Simox_LIB_DIRS "${VirtualRobot_LIBRARY_DIRS}" "${GRASP_STUDIO_LIBRARY_DIRS}" "${SABA_LIBRARY_DIRS}" "${SIMDYNAMICS_LIBRARY_DIRS}")
	#MESSAGE ("Simox_LIB_DIRS: ${Simox_LIB_DIRS}")
SET(Simox_LIB_FLAGS "${VirtualRobot_COMPILE_FLAGS} ${GRASP_STUDIO_COMPILE_FLAGS} ${SABA_COMPILE_FLAGS} ${SIMDYNAMICS_COMPILE_FLAGS}")
	#MESSAGE ("Simox_LIB_FLAGS: ${Simox_LIB_FLAGS}")
SET(Simox_CMAKE_DIR "${Simox_BUILD_DIRECTORY}/share/Simox/cmake")
SET(VirtualRobot_CMAKE_DIR "${Simox_BUILD_DIRECTORY}/share/VirtualRobot/cmake")
SET(Simox_LIBRARY_CMAKE_INCLUDE "${VirtualRobot_CMAKE_INCLUDE}")
SET(Simox_DATA_PATH "${VirtualRobot_DATA_PATH}")

# the SimoxConfig.cmake will be used by external porjects to setup Simox via the FindPackage method.
# So it must be located in the Simox_DIR directory (build-tree -> Simox_BUILD_DIRECTORY)
configure_file(
    ${PROJECT_SOURCE_DIR}/CMakeModules/SimoxConfig.cmake.in
    "${Simox_BUILD_DIRECTORY}/share/Simox/cmake/SimoxConfig.cmake"
    @ONLY)

configure_file(
    ${PROJECT_SOURCE_DIR}/CMakeModules/SimoxConfigVersion.cmake.in
    "${Simox_BUILD_DIRECTORY}/share/Simox/cmake/SimoxConfigVersion.cmake"
    @ONLY)

configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/SimoxExternalLibrarySetup.cmake"
    "${Simox_BUILD_DIRECTORY}/share/Simox/cmake/SimoxExternalLibrarySetup.cmake" COPYONLY)
configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/SimoxMacros.cmake"
    "${Simox_BUILD_DIRECTORY}/share/Simox/cmake/SimoxMacros.cmake" COPYONLY)
#configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/FindSimox.cmake"
#    "${Simox_BUILD_DIRECTORY}/share/Simox/cmake/FindSimox.cmake" COPYONLY)
 
# Create an SimoxConfig.cmake file for the use from the install tree
# and install it

# import Virtual-Robot configs (install)
# do not include files from install tree here (the files are not installed yet. So include them manually from inbuild)
SET (VirtualRobot_DISABLE_DEPENDENCIES_INCLUDE TRUE)
include ("${VirtualRobot_CONFIG_INSTALL_LOCAL}")
include(${PROJECT_SOURCE_DIR}/VirtualRobot/CMakeModules/VirtualRobotExternalLibrarySetup.cmake)
include(${PROJECT_SOURCE_DIR}/VirtualRobot/CMakeModules/VirtualRobotMacros.cmake)

SET(Simox_INCLUDE_DIRS "${VirtualRobot_INCLUDE_DIRS}" "${GRASP_STUDIO_INCLUDE_DIRS}" "${SABA_INCLUDE_DIRS}" "${SIMDYNAMICS_INCLUDE_DIRS}")
SET(Simox_LIB_DIRS "${VirtualRobot_LIBRARY_DIRS}" "${GRASP_STUDIO_LIBRARY_DIRS}" "${SABA_LIBRARY_DIRS}" "${SIMDYNAMICS_LIBRARY_DIRS}")
SET(Simox_LIB_FLAGS "${VirtualRobot_COMPILE_FLAGS} ${GRASP_STUDIO_COMPILE_FLAGS} ${SABA_COMPILE_FLAGS} ${SIMDYNAMICS_COMPILE_FLAGS}")
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
# The SimoxConfig.cmake will be used by external projects to setup Simox via the FindPackage method.
# So it must be located in the Simox_DIR directory (install-tree -> CMAKE_INSTALL_PREFIX)
# changed from DESTINATION "${CMAKE_INSTALL_PREFIX}" to "${Simox_CMAKE_DIR}"
install(FILES
    "${Simox_BUILD_DIRECTORY}/InstallFiles/SimoxConfig.cmake"
    "${Simox_BUILD_DIRECTORY}/InstallFiles/SimoxConfigVersion.cmake"
    DESTINATION "${Simox_CMAKE_DIR}"
    COMPONENT ApplicationData
    )
#removed "${PROJECT_SOURCE_DIR}/CMakeModules/FindSimox.cmake"

install(FILES
    "${PROJECT_SOURCE_DIR}/CMakeModules/SimoxExternalLibrarySetup.cmake"
    "${PROJECT_SOURCE_DIR}/CMakeModules/SimoxMacros.cmake"
    DESTINATION "${Simox_CMAKE_DIR}"
    COMPONENT ApplicationData
    )

#############################################################
# PACKAGING (DEB)

INCLUDE(InstallRequiredSystemLibraries)
SET(CPACK_SET_DESTDIR "on")
SET(CPACK_PACKAGING_INSTALL_PREFIX "/tmp")
SET(CPACK_DEBIAN_PACKAGE_NAME "simox")
SET(CPACK_GENERATOR "DEB")
SET(CPACK_PACKAGE_DESCRIPTION "Simox - A Robot Simulation, Motion and Grasp Planning Toolbox")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Simox - A Robot Simulation, Motion and Grasp Planning Toolbox")
SET(CPACK_PACKAGE_VENDOR "KIT")
SET(CPACK_PACKAGE_CONTACT "Nikolaus Vahrenkamp, <vahrenkamp at kit dot edu>")
SET(CPACK_PACKAGE_VERSION_MAJOR "${Simox_MAJOR_VERSION}")
SET(CPACK_PACKAGE_VERSION_MINOR "${Simox_MINOR_VERSION}")
SET(CPACK_PACKAGE_VERSION_PATCH "${Simox_PATCH_VERSION}")
SET(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}_${Simox_MAJOR_VERSION}.${Simox_MINOR_VERSION}.${Simox_PATCH_VERSION}")
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}_${Simox_MAJOR_VERSION}.${Simox_MINOR_VERSION}.${Simox_PATCH_VERSION}")
 
#SET(CPACK_DEBIAN_PACKAGE_DEPENDS "kdebase-runtime (>= 4:4.2.1), kdelibs5 (>= 4:4.2.1), libc6 (>= 2.1.3), libgcc1 (>= 1:4.1.1), libplasma3, libqt4-dbus (>= 4.5.0), libqtcore4 (>= 4.5.0), libqtgui4 (>= 4.5.0), libstdc++6 (>= 4.2.1)")
#SET(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
#SET(CPACK_DEBIAN_PACKAGE_SECTION "kde")
SET(CPACK_DEBIAN_ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})

#GET_PROPERTY(result GLOBAL ENABLED_FEATURES)
#message(" enabled features: ${result}")
set(CPACK_COMPONENT_APPLICATIONS_DISPLAY_NAME "Simox Examples")
set(CPACK_COMPONENT_APPLICATIONDATA_DISPLAY_NAME "Simox Data")
set(CPACK_COMPONENT_LIBRARIES_DISPLAY_NAME "Simox Libraries")
set(CPACK_COMPONENT_HEADERS_DISPLAY_NAME "Simox C++ Headers")

SET(CPACK_COMPONENTS_ALL Applications Headers Libraries ApplicationData)
INCLUDE(CPack)



