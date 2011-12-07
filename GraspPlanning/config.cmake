GET_FILENAME_COMPONENT (CurrentGSPath ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(GRASPSTUDIO_DIR ${CurrentGSPath})

############################# SETUP PATHS TO Simox #############################
SET(SIMOX_DIR_STANDARD "${CurrentGSPath}/..")
# be sure to have the absolute path
get_filename_component(SIMOX_DIR_STANDARD ${SIMOX_DIR_STANDARD} ABSOLUTE)

SET (GRASPSTUDIO_SimoxDir ${SIMOX_DIR_STANDARD} CACHE STRING "Path to Simox used by GraspStudio")

INCLUDE(${GRASPSTUDIO_SimoxDir}/config.cmake)
INCLUDE(${GRASPSTUDIO_SimoxDir}/SimoxProject.cmake)
	
IF(NOT DEFINED SIMOX_BUILD_DIRECTORY)
	get_filename_component(SIMOX_BUILD_DIRECTORY ${SIMOX_BUILD_DIRECTORY} ABSOLUTE)
	SET(SIMOX_BUILD_DIRECTORY "${GRASPSTUDIO_SimoxDir}/build" CACHE STRING "Simox build directory used by GraspStudio")
	SET(SIMOX_LIB_DIR ${SIMOX_BUILD_DIRECTORY}/lib)
	SET(SIMOX_BIN_DIR ${SIMOX_BUILD_DIRECTORY}/bin)
ENDIF()

############################# SETUP PATHS #############################
ADD_DEFINITIONS(-DGRASPSTUDIO_BASE_DIR="${GRASPSTUDIO_DIR}")

# Define, where to put the binaries
SET(GRASPSTUDIO_LIB_DIR ${SIMOX_LIB_DIR})
SET(GRASPSTUDIO_BIN_DIR ${SIMOX_BIN_DIR})
MESSAGE(STATUS "** GRASPSTUDIO_LIB_DIR: ${GRASPSTUDIO_LIB_DIR}")
MESSAGE(STATUS "** GRASPSTUDIO_BIN_DIR: ${GRASPSTUDIO_BIN_DIR}")


# Define, where to install the binaries
# Define, where to install the binaries
SET(GRASPSTUDIO_INSTALL_LIB_DIR ${SIMOX_INSTALL_LIB_DIR})
SET(GRASPSTUDIO_INSTALL_BIN_DIR ${SIMOX_INSTALL_BIN_DIR})
SET(GRASPSTUDIO_INSTALL_HEADER_DIR ${SIMOX_INSTALL_HEADER_DIR})

#######################################################################
# Setup for testing
#######################################################################
ENABLE_TESTING()
INCLUDE(CTest)

MACRO(ADD_GRASPSTUDIO_TEST TEST_NAME)
	ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
	TARGET_LINK_LIBRARIES(${TEST_NAME} VirtualRobot Saba GraspStudio ${VIRTUAL_ROBOT_LINK_LIBRARIES})
	IF(NOT UNIX)
	   SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${SIMOX_BIN_DIR})
	ENDIF(NOT UNIX)
	ADD_TEST( GraspStudio_${TEST_NAME} ${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO(ADD_GRASPSTUDIO_TEST)
