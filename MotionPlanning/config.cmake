GET_FILENAME_COMPONENT (CurrentSabaPath ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(SABA_DIR ${CurrentSabaPath})

############################# SETUP PATHS TO Simox #############################
SET(SIMOX_DIR_STANDARD "${CurrentSabaPath}/..")
# be sure to have the absolute path
get_filename_component(SIMOX_DIR_STANDARD ${SIMOX_DIR_STANDARD} ABSOLUTE)

SET (SABA_SimoxDir ${SIMOX_DIR_STANDARD} CACHE STRING "Path to Simox used by SaBa")

INCLUDE(${SABA_SimoxDir}/config.cmake)

IF(NOT DEFINED SIMOX_BUILD_DIRECTORY)
	get_filename_component(SIMOX_BUILD_DIRECTORY ${SIMOX_BUILD_DIRECTORY} ABSOLUTE)
	SET(SIMOX_BUILD_DIRECTORY "${SABA_SimoxDir}/build" CACHE STRING "Simox build directory used by SaBa")
	SET(SIMOX_LIB_DIR ${SIMOX_BUILD_DIRECTORY}/lib)
	SET(SIMOX_BIN_DIR ${SIMOX_BUILD_DIRECTORY}/bin)
ENDIF()

############################# SETUP PATHS #############################
ADD_DEFINITIONS(-DSABA_BASE_DIR="${SABA_DIR}")

# Define, where to put the binaries
SET(SABA_LIB_DIR ${SIMOX_LIB_DIR})
SET(SABA_BIN_DIR ${SIMOX_BIN_DIR})
MESSAGE(STATUS "** SABA_LIB_DIR: ${SABA_LIB_DIR}")
MESSAGE(STATUS "** SABA_BIN_DIR: ${SABA_BIN_DIR}")

# Define, where to install the binaries
# Define, where to install the binaries
SET(SABA_INSTALL_LIB_DIR ${SIMOX_INSTALL_LIB_DIR})
SET(SABA_INSTALL_BIN_DIR ${SIMOX_INSTALL_BIN_DIR})
SET(SABA_INSTALL_HEADER_DIR ${SIMOX_INSTALL_HEADER_DIR})

#######################################################################
# Setup for testing
#######################################################################
ENABLE_TESTING()
INCLUDE(CTest)

MACRO(ADD_SABA_TEST TEST_NAME)
	ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
	TARGET_LINK_LIBRARIES(${TEST_NAME} VirtualRobot Saba ${VIRTUAL_ROBOT_LINK_LIBRARIES})
	IF(NOT UNIX)
	   SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${SIMOX_BIN_DIR})
	ENDIF(NOT UNIX)
	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "Saba Tests")
	ADD_TEST( Saba_${TEST_NAME} ${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO(ADD_SABA_TEST)
