
IF (NOT SIMOX_CONFIGURED)

	MESSAGE (STATUS "******** CONFIGURING VirtualRobot StandAlone ********" )
	INCLUDE(CMakeModules/VirtualRobotStandAlone.cmake)

ENDIF(NOT SIMOX_CONFIGURED)


############################# SETUP PATHS #############################
GET_FILENAME_COMPONENT (CurrentVRPath ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(VR_DIR ${CurrentVRPath})
MESSAGE (STATUS "** VR_DIR: ${VR_DIR}")

ADD_DEFINITIONS(-DVR_BASE_DIR="${VR_DIR}")

SET (VIRTUAL_ROBOT_LINK_LIBRARIES ${SIMOX_VISUALIZATION_LIBS} ${Boost_LIBRARIES})

#MESSAGE ("VIRTUAL_ROBOT_LINK_LIBRARIES:${VIRTUAL_ROBOT_LINK_LIBRARIES}")


# Define, where to put the libs and binaries

SET(VR_LIB_DIR ${SIMOX_LIB_DIR})
SET(VR_BIN_DIR ${SIMOX_BIN_DIR})


MESSAGE (STATUS "** VR LIB DIR: ${VR_LIB_DIR}")
MESSAGE (STATUS "** VR BIN DIR: ${VR_BIN_DIR}")

# Define, where to install the binaries
SET(VR_INSTALL_LIB_DIR ${SIMOX_INSTALL_LIB_DIR})
SET(VR_INSTALL_BIN_DIR ${SIMOX_INSTALL_BIN_DIR})
SET(VR_INSTALL_HEADER_DIR ${SIMOX_INSTALL_HEADER_DIR})

# Set OS specific options
IF(UNIX)
	# We are on Linux
	SET(VR_TEST_DIR ${VR_BIN_DIR}/tests)
ELSE(UNIX)
	# We are on Windows
	SET(VR_TEST_DIR ${VR_BIN_DIR})
ENDIF(UNIX)

#######################################################################
# Setup for testing
#######################################################################
ENABLE_TESTING()
INCLUDE(CTest)

MESSAGE(STATUS "** Test output directory: ${VR_TEST_DIR}")

MACRO(ADD_VR_TEST TEST_NAME)
    
    INCLUDE_DIRECTORIES(${SIMOX_VISUALIZATION_INCLUDE_PATHS})
    ADD_DEFINITIONS(${SIMOX_VISUALIZATION_COMPILE_FLAGS})
	ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
	TARGET_LINK_LIBRARIES(${TEST_NAME} VirtualRobot ${VIRTUAL_ROBOT_LINK_LIBRARIES})
	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${VR_TEST_DIR})
	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "VirtualRobot Tests")
	ADD_TEST(NAME VirtualRobot_${TEST_NAME}
	         COMMAND ${VR_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO(ADD_VR_TEST)

#######################################################################
# Setup for installation
#######################################################################

############################################
MESSAGE(STATUS "Generating CMake files")

set(VIRTUAL_ROBOT_MAJOR_VERSION 0)
set(VIRTUAL_ROBOT_MINOR_VERSION 1)
set(VIRTUAL_ROBOT_PATCH_VERSION 0)
set(VIRTUAL_ROBOT_VERSION
    ${VIRTUAL_ROBOT_MAJOR_VERSION}.${VIRTUAL_ROBOT_MINOR_VERSION}.${VIRTUAL_ROBOT_PATCH_VERSION})

set(VIRTUAL_ROBOT_LIBRARIES VirtualRobot ColCheckerPQP)
set(VIRTUAL_ROBOT_EXECUTABLES "")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
EXPORT(PACKAGE VirtualRobot)
 
# Install the export set for use with the install-tree
install(
    EXPORT VirtualRobotLibraryDepends
    DESTINATION "${CMAKE_INSTALL_PREFIX}/share/VirtualRobot/cmake")


# Create an VirtualRobotConfig.cmake file for the use from the build tree
SET(VIRTUAL_ROBOT_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/..")
SET(VIRTUAL_ROBOT_LIB_DIRS "${PROJECT_BINARY_DIR}/${LIB_DIR}")
SET(VIRTUAL_ROBOT_CMAKE_DIR "${PROJECT_BINARY_DIR}")
configure_file(
    ${VR_DIR}/CMakeModules/VirtualRobotConfig.cmake.in
    "${PROJECT_BINARY_DIR}/VirtualRobotConfig.cmake"
    @ONLY)
configure_file(
    ${VR_DIR}/CMakeModules/VirtualRobotConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/VirtualRobotConfigVersion.cmake"
    @ONLY)

# Create an VirtualRobotConfig.cmake file for the use from the install tree
# and install it
SET(VIRTUAL_ROBOT_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
SET(VIRTUAL_ROBOT_LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${LIB_DIR}")
SET(VIRTUAL_ROBOT_CMAKE_DIR "${CMAKE_INSTALL_PREFIX}/share/VirtualRobot/cmake")
configure_file(
    ${VR_DIR}/CMakeModules/VirtualRobotConfig.cmake.in
    "${PROJECT_BINARY_DIR}/InstallFiles/VirtualRobotConfig.cmake"
    @ONLY)
configure_file(
    ${VR_DIR}/CMakeModules/VirtualRobotConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/InstallFiles/VirtualRobotConfigVersion.cmake"
    @ONLY)
install(FILES
    "${PROJECT_BINARY_DIR}/InstallFiles/VirtualRobotConfig.cmake"
    "${PROJECT_BINARY_DIR}/InstallFiles/VirtualRobotConfigVersion.cmake"
    DESTINATION "${VIRTUAL_ROBOT_CMAKE_DIR}")
