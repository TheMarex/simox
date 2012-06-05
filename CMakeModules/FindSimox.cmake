# Find SIMOX 
#
# This find script searches for simox and includes all neccessary config files.
#
# Search order
##	1. ${Custom_Simox_DIR}
##   2. ${Simox_DIR}
##   3. $ENV{Simox_DIR}
##   [ The paths should point to either the build or the install directory or Simox ]
#
# The following variables are set:
## Simox_FOUND       - TRUE on success
## Simox_INCLUDE_DIR - The include directory
## Simox_LIBRARIES   - The libraries
#
#
# A CMakeLists.txt file for setting up a simox related project could look like this:
#
## PROJECT ( myDemo )
## FIND_PACKAGE(Simox REQUIRED)
## IF(Simox_USE_COIN_VISUALIZATION)
##   FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/myDemo.cpp ${PROJECT_SOURCE_DIR}/myWindow.cpp)
##   FILE(GLOB INCS ${PROJECT_SOURCE_DIR}/myWindow.h)
##   set(GUI_MOC_HDRS ${PROJECT_SOURCE_DIR}/myWindow.h)
##   set(GUI_UIS ${PROJECT_SOURCE_DIR}/myWindow.ui)
##   SimoxQtApplication(${PROJECT_NAME} "${SRCS}" "${INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
## ENDIF()

#### CMAKE CONFIG

find_path( Simox_DIR SimoxConfig.cmake 
                    "${Custom_Simox_DIR}"
                    "${Simox_DIR}" 
                    "$ENV{Simox_DIR}")

SET (Simox_CMAKE_CONFIG ${Simox_DIR}/SimoxConfig.cmake)
if (NOT (Simox_FIND_QUIETLY OR VirtualRobot_FIND_QUIETLY))       
    MESSAGE(STATUS " * Including ${Simox_CMAKE_CONFIG}")
endif()
include (${Simox_CMAKE_CONFIG})

if( Simox_LIBRARIES AND Simox_BASE_DIR)
    set( Simox_FOUND TRUE )
    set( VirtualRobot_FOUND TRUE )
    set( Simox_INCLUDE_DIR ${Simox_BASE_DIR} )
    set( Simox_LIBRARY ${Simox_LIBRARIES} )
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( Simox DEFAULT_MSG Simox_LIBRARIES Simox_INCLUDE_DIR )

mark_as_advanced( Simox_INCLUDE_DIR Simox_LIBRARIES )
