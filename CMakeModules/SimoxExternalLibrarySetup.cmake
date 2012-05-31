
# Setup paths, libs and external cmake files to be used for SIMOX

MESSAGE(STATUS "SETTING LIBRARY DEPENDENCIES FOR SIMOX")

# VR

INCLUDE (${Simox_DIR}/VirtualRobotConfig.cmake)
INCLUDE (${VIRTUAL_ROBOT_CMAKE_DIR}/VirtualRobotExternalLibrarySetup.cmake)

# SABA

# GRASP STUDIO

