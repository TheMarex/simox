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
## SIMOX_FOUND       - TRUE on success
## SIMOX_INCLUDE_DIR - The include directory
## SIMOX_LIBRARIES   - The libraries
#
#
# A CMakeLists.txt file for setting up a simox related project could look like this:
#
## PROJECT ( myDemo )
## SET(Simox_DIR $ENV{Simox_DIR} CACHE STRING "Choose the path to Simox (install or build).")
## FIND_PACKAGE(Simox REQUIRED)
## IF(SIMOX_USE_COIN_VISUALIZATION)
##   FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/myDemo.cpp ${PROJECT_SOURCE_DIR}/myWindow.cpp)
##   FILE(GLOB INCS ${PROJECT_SOURCE_DIR}/myWindow.h)
##   set(GUI_MOC_HDRS ${PROJECT_SOURCE_DIR}/myWindow.h)
##   set(GUI_UIS ${PROJECT_SOURCE_DIR}/myWindow.ui)
##   SimoxQtApplication(${PROJECT_NAME} "${SRCS}" "${INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
## ENDIF()

 
MESSAGE ("START1")

#### CMAKE CONFIG
find_file( Simox_CMAKE_CONFIG SimoxConfig.cmake 
                    "${Custom_Simox_DIR}/share/Simox/cmake" 
                    "${Simox_DIR}/share/Simox/cmake" 
                    "$ENV{Simox_DIR}/share/Simox/cmake"
                    "${Custom_Simox_DIR}"
                    "${Simox_DIR}" 
                    "$ENV{Simox_DIR}")

MESSAGE(STATUS " * Including ${Simox_CMAKE_CONFIG}")
include (${Simox_CMAKE_CONFIG})

if( SIMOX_LIBRARIES AND SIMOX_BASE_DIR)
    set( SIMOX_FOUND TRUE )
    set( SIMOX_INCLUDE_DIR ${SIMOX_BASE_DIR} )
    set( SIMOX_LIBRARY ${SIMOX_LIBRARIES} )
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( Simox DEFAULT_MSG SIMOX_LIBRARIES SIMOX_INCLUDE_DIR )

mark_as_advanced( SIMOX_INCLUDE_DIR SIMOX_LIBRARIES )
