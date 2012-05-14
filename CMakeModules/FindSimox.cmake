# Find SIMOX 
#
# SIMOX_FOUND       - TRUE on success
# SIMOX_INCLUDE_DIR - The include directory
# SIMOX_LIBRARIES   - The libraries
#
# Search order
#   1. ${Simox_DIR}
#   2. $ENV{Simox_DIR}
#   [ The paths should point to either the build or the install directory or Simox ]

# Here a custom search directory can be manually set.
#SET(Custom_Simox_DIR "C:/Projects/libs/Simox")

 
#### CMAKE CONFIG
find_file( Simox_CMAKE_CONFIG SimoxConfig.cmake 
                    "${Custom_Simox_DIR}/share/Simox/cmake" 
                    "${Simox_DIR}/share/Simox/cmake" 
                    "$ENV{Simox_DIR}/share/Simox/cmake")
MESSAGE(STATUS "Simox_CMAKE_CONFIG: ${Simox_CMAKE_CONFIG}")
include (${Simox_CMAKE_CONFIG})

if( SIMOX_LIBRARIES AND SIMOX_BASE_DIR)
    set( SIMOX_FOUND TRUE )
    set( SIMOX_INCLUDE_DIR ${SIMOX_BASE_DIR} )
    set( SIMOX_LIBRARY ${SIMOX_LIBRARIES} )
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( Simox DEFAULT_MSG SIMOX_LIBRARIES SIMOX_INCLUDE_DIR )

mark_as_advanced( SIMOX_INCLUDE_DIR SIMOX_LIBRARIES )

