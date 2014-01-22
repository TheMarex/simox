PROJECT(Simox)

cmake_minimum_required(VERSION 2.8.3)
if(NOT ("${CMAKE_VERSION}" VERSION_LESS 2.8.12))
	cmake_policy(SET CMP0022 OLD) # avoid INTERFACE_LINK_LIBRARIES warnings
ENDIF()

MESSAGE(STATUS "******************** Configuring Simox ************************")
set (Simox_BUILD_DIRECTORY ${CMAKE_BINARY_DIR} CACHE STRING "Simox build directory")
#set (Simox_BUILD_VirtualRobot TRUE CACHE BOOL "Build Simox VirtualRobot")
set (Simox_BUILD_Saba TRUE CACHE BOOL "Build Motion Planning library")
set (Simox_BUILD_GraspStudio TRUE CACHE BOOL "Build Grasp Planning library")
set (Simox_BUILD_SimDynamics FALSE CACHE BOOL "Build Dynamic Simulation")

INCLUDE (config.cmake)


MESSAGE(STATUS "** Simox build directory: ${Simox_BUILD_DIRECTORY}")
add_subdirectory(VirtualRobot)
set (SIMOX_EXPORT_TARGET_LIST VirtualRobot)
list (APPEND Simox_LIBRARIES VirtualRobot)
list(APPEND SIMOX_EXPORT_TARGET_LIST ColCheckerPQP)
if (Simox_BUILD_Saba)
    add_subdirectory(MotionPlanning)
    list(APPEND SIMOX_EXPORT_TARGET_LIST Saba)
list (APPEND Simox_LIBRARIES Saba)
endif()
if (Simox_BUILD_GraspStudio)
    add_subdirectory(GraspPlanning)
    list(APPEND SIMOX_EXPORT_TARGET_LIST GraspStudio)
    list(APPEND SIMOX_EXPORT_TARGET_LIST qhull)
    list(APPEND SIMOX_EXPORT_TARGET_LIST powercrust)
	list (APPEND Simox_LIBRARIES GraspStudio)
endif()
if (Simox_BUILD_SimDynamics)
    add_subdirectory(SimDynamics)
    list(APPEND SIMOX_EXPORT_TARGET_LIST SimDynamics)
	list (APPEND Simox_LIBRARIES SimDynamics)
	list (APPEND Simox_EXTERNAL_LIBRARIES ${Simox_SimDynamics_EXTERNAL_LIBRARIES})
	list (APPEND Simox_EXTERNAL_INCLUDE_DIRS ${Simox_SimDynamics_INCLUDE_DIRS})



endif()

  MESSAGE(STATUS "** Simox_EXTERNAL_INCLUDE_DIRS:${Simox_EXTERNAL_INCLUDE_DIRS}")
  MESSAGE(STATUS "** Simox_EXTERNAL_LIBRARY_DIRS:${Simox_EXTERNAL_LIBRARY_DIRS}")
  MESSAGE(STATUS "** Simox_EXTERNAL_LIBRARIES:${Simox_EXTERNAL_LIBRARIES}")
  MESSAGE(STATUS "** Simox_EXTERNAL_LIBRARY_FLAGS:${Simox_EXTERNAL_LIBRARY_FLAGS}")
  MESSAGE(STATUS "** Simox_EXTERNAL_CMAKE_INCLUDE:${Simox_EXTERNAL_CMAKE_INCLUDE}")

add_subdirectory(doc)


#######################################################################
# Setup for installation
#######################################################################

############################################
MESSAGE (STATUS "** Simox: Generating CMake files for build and install setup")


# Add all targets to the build-tree export set

export(TARGETS ${SIMOX_EXPORT_TARGET_LIST}
  FILE "${PROJECT_BINARY_DIR}/SimoxTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE Simox)

# Create the SimoxConfig.cmake and SimoxVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
set (Simox_DATA_PATH ${Simox_DATA_DIR})
set (Simox_MACROS_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/CMakeModules")
configure_file(CMakeModules/SimoxConfig.cmake.in
  "${PROJECT_BINARY_DIR}/SimoxConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${Simox_CMAKE_DIR}/${REL_INCLUDE_DIR}")
set (Simox_DATA_PATH ${Simox_INSTALL_DATA_DIR})
set (Simox_MACROS_INCLUDE_DIR "\${Simox_CMAKE_DIR}")
configure_file(CMakeModules/SimoxConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/SimoxConfig.cmake" @ONLY)
# ... for both
configure_file(CMakeModules/SimoxConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/SimoxConfigVersion.cmake" @ONLY)

# Install the SimoxConfig.cmake and SimoxConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/SimoxConfig.cmake"
  "${PROJECT_BINARY_DIR}/SimoxConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

install(FILES
    "${PROJECT_SOURCE_DIR}/CMakeModules/SimoxMacros.cmake"
    DESTINATION "${INSTALL_CMAKE_DIR}"
    COMPONENT ApplicationData
    )


# Install the export set for use with the install-tree
install(EXPORT SimoxTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)


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



