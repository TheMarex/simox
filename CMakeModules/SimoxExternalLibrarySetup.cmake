
# Setup paths, libs and external cmake files to be used for SIMOX

if (NOT (Simox_FIND_QUIETLY OR VirtualRobot_FIND_QUIETLY))
    MESSAGE(STATUS "SETTING LIBRARY DEPENDENCIES FOR SIMOX")
endif()

# VR

INCLUDE (${VirtualRobot_CMAKE_DIR}/VirtualRobotConfig.cmake)
INCLUDE (${VirtualRobot_CMAKE_DIR}/VirtualRobotExternalLibrarySetup.cmake)

# SABA

# GRASP STUDIO

