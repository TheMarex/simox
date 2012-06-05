# Find VirtualRobot 
#
# This find script searches for VirtualRobot and includes all neccessary config files.
#
# VirtualRobot_FOUND       - TRUE on success
# VirtualRobot_INCLUDE_DIR - The include directory
# VirtualRobot_LIBRARIES   - The libraries
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
## FIND_PACKAGE(VirtualRobot REQUIRED)
## IF(VirtualRobot_USE_COIN_VISUALIZATION)
##   FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/myDemo.cpp ${PROJECT_SOURCE_DIR}/myWindow.cpp)
##   FILE(GLOB INCS ${PROJECT_SOURCE_DIR}/myWindow.h)
##   set(GUI_MOC_HDRS ${PROJECT_SOURCE_DIR}/myWindow.h)
##   set(GUI_UIS ${PROJECT_SOURCE_DIR}/myWindow.ui)
##   VirtualRobotQtApplication(${PROJECT_NAME} "${SRCS}" "${INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
## ENDIF()


 
#### CMAKE CONFIG
find_path( VirtualRobot_DIR VirtualRobotConfig.cmake 
                    "${Custom_Simox_VR_DIR}"
                    "${VirtualRobot_DIR}" 
                    "$ENV{VirtualRobot_DIR}"
                    "${Simox_DIR}" 
                    "$ENV{SImox_DIR}"
                    )


SET (Simox_VR_CMAKE_CONFIG ${VirtualRobot_DIR}/VirtualRobotConfig.cmake) 
if (NOT (Simox_FIND_QUIETLY OR VirtualRobot_FIND_QUIETLY)) 
    MESSAGE(STATUS " Including ${Simox_VR_CMAKE_CONFIG}")
endif()
include (${Simox_VR_CMAKE_CONFIG})

if( VirtualRobot_LIBRARIES AND VirtualRobot_DIR)
    set( VirtualRobot_FOUND TRUE )
    set( VirtualRobot_INCLUDE_DIR ${VirtualRobot_DIR} )
    set( VirtualRobot_LIBRARY ${VirtualRobot_LIBRARIES} )
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( VirtualRobot DEFAULT_MSG VirtualRobot_LIBRARIES VirtualRobot_INCLUDE_DIR )

mark_as_advanced( VirtualRobot_INCLUDE_DIR VirtualRobot_LIBRARIES )

