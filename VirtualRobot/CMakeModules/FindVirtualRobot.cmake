# Find VIRTUAL_ROBOT 
#
# VIRTUAL_ROBOT_FOUND       - TRUE on success
# VIRTUAL_ROBOT_INCLUDE_DIR - The include directory
# VIRTUAL_ROBOT_LIBRARIES   - The libraries
#
# Search order
#   1. ${VirtualRobot_DIR}
#   2. $ENV{VirtualRobot_DIR}
#   3. ${Simox_DIR}
#   4. $ENV{Simox_DIR}
#   [ The paths should point to either the build or the install directory of Simox/VirtualRobot ]

# Here a custom search directory can be manually set.
#SET(Custom_Simox_VR_DIR "C:/Projects/libs/VirtualRobot_standalone")

 
#### CMAKE CONFIG
find_file( Simox_VR_CMAKE_CONFIG VirtualRobotConfig.cmake 
                    "${Custom_Simox_VR_DIR}/share/VirtualRobot/cmake" 
                    "${VirtualRobot_DIR}/share/VirtualRobot/cmake" 
                    "$ENV{VirtualRobot_DIR}/share/VirtualRobot/cmake" 
                    "${Simox_DIR}/share/VirtualRobot/cmake" 
                    "$ENV{Simox_DIR}/share/VirtualRobot/cmake")
MESSAGE(STATUS " Simox_VR_CMAKE_CONFIG: ${Simox_VR_CMAKE_CONFIG}")
include (${Simox_VR_CMAKE_CONFIG})

if( VIRTUAL_ROBOT_LIBRARIES AND VIRTUAL_ROBOT_DIR)
    set( VIRTUAL_ROBOT_FOUND TRUE )
    set( VIRTUAL_ROBOT_INCLUDE_DIR ${VIRTUAL_ROBOT_DIR} )
    set( VIRTUAL_ROBOT_LIBRARY ${VIRTUAL_ROBOT_LIBRARIES} )
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( Simox DEFAULT_MSG VIRTUAL_ROBOT_LIBRARIES VIRTUAL_ROBOT_INCLUDE_DIR )

mark_as_advanced( VIRTUAL_ROBOT_INCLUDE_DIR VIRTUAL_ROBOT_LIBRARIES )

