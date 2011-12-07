
IF (NOT SIMOX_CONFIGURED)


	MESSAGE (STATUS "******** CONFIGURING VirtualRobot StandAlone ********" )
	SET(SIMOX_CONFIGURED TRUE)
	
	# Set up for debug build
	IF(NOT CMAKE_BUILD_TYPE)
	  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
	      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
	      FORCE)
	ENDIF(NOT CMAKE_BUILD_TYPE)
	
	
	
	
	############################# SETUP MODULES #############################
	GET_FILENAME_COMPONENT (CurrentPath ${CMAKE_CURRENT_LIST_FILE} PATH)
	SET(CMAKE_MODULE_PATH ${CurrentPath}/CMakeModules)
	
	#### Eigen
	FIND_PACKAGE (Eigen3 REQUIRED)
	if (Eigen3_FOUND)
		MESSAGE (STATUS "** Eigen3 found at: ${Eigen3_INCLUDE_DIR}")
		INCLUDE_DIRECTORIES(${Eigen3_INCLUDE_DIR})
	endif (Eigen3_FOUND)
	
	#### BOOST
	FIND_PACKAGE(Boost 1.42.0 COMPONENTS filesystem system unit_test_framework program_options REQUIRED)
	if (Boost_FOUND)
	    MESSAGE (STATUS "Boost found at: ${Boost_INCLUDE_DIR}")
	    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
	    LINK_DIRECTORIES (${Boost_LIBRARY_DIRS})
	    # disable boost auto linking
	    add_definitions( -DBOOST_ALL_NO_LIB )
	    add_definitions( -DBOOST_PROGRAM_OPTIONS_DYN_LINK )
	    add_definitions( -DBOOST_FILESYSTEM_DYN_LINK )
	    add_definitions( -DBOOST_SYSTEM_DYN_LINK )
	    add_definitions( -DBOOST_UNIT_TEST_FRAMEWORK_DYN_LINK )
        #add_definitions( -DBOOST_LIB_DIAGNOSTIC )
	else (Boost_FOUND)
	    MESSAGE ("!! Could not find Boost !!")
	endif (Boost_FOUND)
	
	#### Coin3D (Qt and SoQt)
	OPTION(SIMOX_USE_COIN_VISUALIZATION "Use Coin3D for visualization" ON)
	if (SIMOX_USE_COIN_VISUALIZATION)
		FIND_PACKAGE(Coin3D REQUIRED)
		if (COIN3D_FOUND)
			MESSAGE (STATUS "** Coin3D found at: ${COIN3D_INCLUDE_DIRS}")
			INCLUDE_DIRECTORIES(${COIN3D_INCLUDE_DIRS})
			ADD_DEFINITIONS(-DCOIN_DLL)
		endif (COIN3D_FOUND)
		
		#### QT 
		# QT_QMAKE_EXECUTABLE is the only relieable way of setting the qt4 path!
		# convert env var to cmake define		
		file(TO_CMAKE_PATH "$ENV{QT_QMAKE_EXECUTABLE}" QT_QMAKE_EXECUTABLE)
 		FIND_PACKAGE(Qt4 4.6.0 COMPONENTS QtOpenGL QtCore QtGui)
		if ( QT_FOUND )
			MESSAGE (STATUS "** Qt4 found at: " ${QT_INCLUDE_DIR})
			include(${QT_USE_FILE})
				
			#### SoQt
			# This will set SoQt_INCLUDE_DIRS and SoQt_LIBRARIES
			FIND_PACKAGE(SoQt)
			if (SOQT_FOUND)
				MESSAGE (STATUS "** SoQt found at:" ${SoQt_INCLUDE_DIRS})
				ADD_DEFINITIONS(-DSOQT_DLL)
			else (SOQT_FOUND)
				MESSAGE (STATUS "-> Did not found SoQt. Disabling SoQt support.")
			endif (SOQT_FOUND)
		else ( QT_FOUND )
			MESSAGE (STATUS "-> Did not found Qt. Disabling Qt/SoQt support.")
		endif ( QT_FOUND )
	endif (SIMOX_USE_COIN_VISUALIZATION)
	
	############################# SETUP PATHS #############################
	IF(DEFINED SIMOX_DIR)
		get_filename_component(SIMOX_DIR ${SIMOX_DIR} ABSOLUTE)
	ELSE()
		SET(SIMOX_DIR "${CurrentPath}/..") # we assume simox a directory above?!
	ENDIF()
	MESSAGE (STATUS "SIMOX_DIR: ${SIMOX_DIR}")
	
	ADD_DEFINITIONS(-DSIMOX_BASE_DIR="${SIMOX_DIR}")
	
	SET(BIN_DIR bin)
	SET(LIB_DIR lib)
	
	IF(DEFINED SIMOX_BUILD_DIRECTORY)
		get_filename_component(SIMOX_BUILD_DIRECTORY ${SIMOX_BUILD_DIRECTORY} ABSOLUTE)
		MESSAGE (STATUS "** SIMOX Build dir defined: ${SIMOX_BUILD_DIRECTORY}")
	ELSE()
		SET(SIMOX_BUILD_DIRECTORY ${CMAKE_BINARY_DIR})
		MESSAGE (STATUS "** SIMOX Build dir not defined, using CMAKE_BINARY_DIR: ${SIMOX_BUILD_DIRECTORY}")
	ENDIF()
	
	SET(SIMOX_LIB_DIR ${SIMOX_BUILD_DIRECTORY}/${LIB_DIR})
	SET(SIMOX_BIN_DIR ${SIMOX_BUILD_DIRECTORY}/${BIN_DIR})
	
	MESSAGE (STATUS "** SIMOX LIB DIR: ${SIMOX_LIB_DIR}")
	MESSAGE (STATUS "** SIMOX BIN DIR: ${SIMOX_BIN_DIR}")
	
	SET(SIMOX_INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/${LIB_DIR})
	SET(SIMOX_INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/${BIN_DIR})
	SET(SIMOX_INSTALL_HEADER_DIR ${CMAKE_INSTALL_PREFIX}/include/)
	
	############################# Set OS specific options #############################
	IF(UNIX)
		# We are on Linux
		IF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
			ADD_DEFINITIONS(-fPIC)
		ENDIF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
	
		IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
			MESSAGE(STATUS "Configuring Debug build")
			ADD_DEFINITIONS(-D_DEBUG) # -Wall -W -Werror -pedantic)
		ELSE()
			MESSAGE(STATUS "Configuring Release build")
		ENDIF()
	ELSE(UNIX)
		# We are on Windows
		ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
		
		# On MSVC we compile with /MP flag (use multiple threads)
		IF(MSVC)
			ADD_DEFINITIONS(/MP)
		ENDIF(MSVC)
	ENDIF(UNIX)
	
	
	# Allow #include <VirtualRobot/*.h>
	INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${SIMOX_DIR})

ENDIF(NOT SIMOX_CONFIGURED)


############################# SETUP PATHS #############################
GET_FILENAME_COMPONENT (CurrentVRPath ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(VR_DIR ${CurrentVRPath})
MESSAGE (STATUS "** VR_DIR: ${VR_DIR}")

ADD_DEFINITIONS(-DVR_BASE_DIR="${VR_DIR}")

IF(SIMOX_USE_COIN_VISUALIZATION)
	SET (VIRTUAL_ROBOT_LINK_LIBRARIES ${COIN3D_LIBRARIES} ${Boost_LIBRARIES})
ELSE(SIMOX_USE_COIN_VISUALIZATION)
	SET (VIRTUAL_ROBOT_LINK_LIBRARIES ${Boost_LIBRARIES})
ENDIF(SIMOX_USE_COIN_VISUALIZATION)
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
	ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
	TARGET_LINK_LIBRARIES(${TEST_NAME} VirtualRobot ${VIRTUAL_ROBOT_LINK_LIBRARIES})
	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${VR_TEST_DIR})
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
