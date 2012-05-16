# Find VIRTUAL_ROBOT 
#
# This find script searches for VirtualRobot and includes all neccessary config files.
#
# VIRTUAL_ROBOT_FOUND       - TRUE on success
# VIRTUAL_ROBOT_INCLUDE_DIR - The include directory
# VIRTUAL_ROBOT_LIBRARIES   - The libraries
#
# Search order
##  1. ${Custom_Simox_VR_DIR}
##   2. ${VirtualRobot_DIR}
##   3. $ENV{VirtualRobot_DIR}
##   4. ${Simox_DIR}
##   5. $ENV{Simox_DIR}
##   [ The paths should point to either the build or the install directory of Simox/VirtualRobot ]
#
# A CMakeLists.txt file for setting up a VirtualRobot related project could look like this:
#
## PROJECT ( myDemo )
## SET(VirtualRobot_DIR $ENV{Simox_DIR} CACHE STRING "Choose the path to VirtualRobot (install or build).")
## SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${VirtualRobot_DIR}/share/VirtualRobot/cmake)
## FIND_PACKAGE(VirtualRobot REQUIRED)
## IF(VIRTUAL_ROBOT_USE_COIN_VISUALIZATION)
##   FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/myDemo.cpp ${PROJECT_SOURCE_DIR}/myWindow.cpp)
##   FILE(GLOB INCS ${PROJECT_SOURCE_DIR}/myWindow.h)
##   set(GUI_MOC_HDRS ${PROJECT_SOURCE_DIR}/myWindow.h)
##   set(GUI_UIS ${PROJECT_SOURCE_DIR}/myWindow.ui)
##   VirtualRobotQtApplication(${PROJECT_NAME} "${SRCS}" "${INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
## ENDIF()


 
#### CMAKE CONFIG
find_file( Simox_VR_CMAKE_CONFIG VirtualRobotConfig.cmake 
                    "${Custom_Simox_VR_DIR}/share/VirtualRobot/cmake" 
                    "${VirtualRobot_DIR}/share/VirtualRobot/cmake" 
                    "$ENV{VirtualRobot_DIR}/share/VirtualRobot/cmake" 
                    "${Simox_DIR}/share/VirtualRobot/cmake" 
                    "$ENV{Simox_DIR}/share/VirtualRobot/cmake")
MESSAGE(STATUS " Including ${Simox_VR_CMAKE_CONFIG}")
include (${Simox_VR_CMAKE_CONFIG})

if( VIRTUAL_ROBOT_LIBRARIES AND VIRTUAL_ROBOT_DIR)
    
    set( VIRTUAL_ROBOT_FOUND TRUE )
    set( VIRTUAL_ROBOT_INCLUDE_DIR ${VIRTUAL_ROBOT_DIR} )
    set( VIRTUAL_ROBOT_LIBRARY ${VIRTUAL_ROBOT_LIBRARIES} )
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( Simox DEFAULT_MSG VIRTUAL_ROBOT_LIBRARIES VIRTUAL_ROBOT_INCLUDE_DIR )

mark_as_advanced( VIRTUAL_ROBOT_INCLUDE_DIR VIRTUAL_ROBOT_LIBRARIES )

