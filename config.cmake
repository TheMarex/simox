
IF (NOT SIMOX_CONFIGURED)

	# defines SIMOX_CONFIGURED variable which indicates that this config file has already been included
	SET(SIMOX_CONFIGURED TRUE)
	
	# Set up for debug build
	IF(NOT CMAKE_BUILD_TYPE)
	  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
	      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
	      FORCE)
	ENDIF(NOT CMAKE_BUILD_TYPE)
	
	############################# SETUP MODULES #############################
	GET_FILENAME_COMPONENT (CurrentPath ${CMAKE_CURRENT_LIST_FILE} PATH)
	#MESSAGE (STATUS "module path: "  ${CurrentPath}/CMakeModules)
	SET(CMAKE_MODULE_PATH ${CurrentPath}/CMakeModules)
	
	#### Eigen
	FIND_PACKAGE (Eigen3 REQUIRED)
	if (Eigen3_FOUND)
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
			INCLUDE_DIRECTORIES(${COIN3D_INCLUDE_DIRS})
			ADD_DEFINITIONS(-DCOIN_DLL)
		endif (COIN3D_FOUND)
		
		#### QT 
		# QT_QMAKE_EXECUTABLE is the only relieable way of setting the qt4 path!
		# convert env var to cmake define	
		IF(NOT "$ENV{QT_QMAKE_EXECUTABLE}" STREQUAL "")
		    MESSAGE (STATUS "USING QT-PATH from environment variable QT_QMAKE_EXECUTABLE: $ENV{QT_QMAKE_EXECUTABLE}")
		    file(TO_CMAKE_PATH "$ENV{QT_QMAKE_EXECUTABLE}" QT_QMAKE_EXECUTABLE)
		ENDIF()
		FIND_PACKAGE(Qt4 4.6.0 COMPONENTS QtOpenGL QtCore QtGui)
		if ( QT_FOUND )
			MESSAGE (STATUS "Found Qt4: " ${QT_INCLUDE_DIR})
			include(${QT_USE_FILE})
			#MESSAGE(STATUS "QT_LIBRARIES: " ${QT_LIBRARIES})
	
			#### SoQt
			# This will set SoQt_INCLUDE_DIRS and SoQt_LIBRARIES
			FIND_PACKAGE(SoQt)
			if (SOQT_FOUND)
				MESSAGE (STATUS "Found SoQt:" ${SoQt_INCLUDE_DIRS})
				ADD_DEFINITIONS(-DSOQT_DLL)
			else (SOQT_FOUND)
				MESSAGE (STATUS "Did not found SoQt. Disabling SoQt support.")
			endif (SOQT_FOUND)
		else ( QT_FOUND )
			MESSAGE (STATUS "Did not found Qt. Disabling Qt/SoQt support.")
		endif ( QT_FOUND )
	endif (SIMOX_USE_COIN_VISUALIZATION)
	
	############################# SETUP PATHS #############################
	IF(DEFINED SIMOX_DIR)
			get_filename_component(SIMOX_DIR ${SIMOX_DIR} ABSOLUTE)
	ELSE()
		SET(SIMOX_DIR ${CurrentPath})
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