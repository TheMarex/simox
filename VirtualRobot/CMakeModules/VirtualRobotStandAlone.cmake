SET(SIMOX_CONFIGURED TRUE)

# Set up for debug build
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)


############################# SETUP MODULES #############################
GET_FILENAME_COMPONENT (CurrentPath ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CurrentPath}/../CMakeModules)

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

#### QT 
# QT_QMAKE_EXECUTABLE is the only relieable way of setting the qt4 path!
# convert env var to cmake define	
IF(NOT "$ENV{QT_QMAKE_EXECUTABLE}" STREQUAL "")
    MESSAGE (STATUS "USING QT-PATH from environment variable QT_QMAKE_EXECUTABLE: $ENV{QT_QMAKE_EXECUTABLE}")
    file(TO_CMAKE_PATH "$ENV{QT_QMAKE_EXECUTABLE}" QT_QMAKE_EXECUTABLE)
ENDIF()
FIND_PACKAGE(Qt4 4.6.0 COMPONENTS QtOpenGL QtCore QtGui)

#### VISUALIZATION Coin3D+Qt+SoQt / OSG+Qt
##########################################
SET (SIMOX_VISUALIZATION FALSE)
SET (SIMOX_VISUALIZATION_LIBS "")
SET (SIMOX_VISUALIZATION_INCLUDE_PATHS "")
SET (SIMOX_VISUALIZATION_COMPILE_FLAGS "")

OPTION(SIMOX_USE_COIN_VISUALIZATION "Use Coin3D for visualization" ON)
OPTION(SIMOX_USE_OPENSCENEGRAPH_VISUALIZATION "Use OpenSceneGraph for visualization" OFF)


if (SIMOX_USE_COIN_VISUALIZATION)
    MESSAGE(STATUS "Searching Coin3D, Qt and SoQt...")
    
    ##### Coin3D
	FIND_PACKAGE(Coin3D REQUIRED)
	if (COIN3D_FOUND)
	    MESSAGE (STATUS "Found Coin3D: " ${COIN3D_INCLUDE_DIRS})
		##INCLUDE_DIRECTORIES(${COIN3D_INCLUDE_DIRS})
		##ADD_DEFINITIONS(-DCOIN_DLL)
	endif (COIN3D_FOUND)
	

	if ( QT_FOUND )
		MESSAGE (STATUS "Found Qt4: " ${QT_INCLUDE_DIR})
		include(${QT_USE_FILE})
		#MESSAGE(STATUS "QT_LIBRARIES: " ${QT_LIBRARIES})

		#### SoQt
		# This will set SoQt_INCLUDE_DIRS and SoQt_LIBRARIES
		FIND_PACKAGE(SoQt)
		if (SOQT_FOUND)
			MESSAGE (STATUS "Found SoQt:" ${SoQt_INCLUDE_DIRS})
			##ADD_DEFINITIONS(-DSOQT_DLL)
		else (SOQT_FOUND)
			MESSAGE (STATUS "Did not found SoQt. Disabling SoQt support.")
		endif (SOQT_FOUND)
	else ( QT_FOUND )
		MESSAGE (STATUS "Did not found Qt. Disabling Qt/SoQt support.")
	endif ( QT_FOUND )
	
	if (QT_FOUND AND SOQT_FOUND AND COIN3D_FOUND)
	    MESSAGE (STATUS "Enabling Coin3D/Qt/SoQt support")
	    MESSAGE (STATUS "By using the Con3D library, the license of Simox is not LGPL any more. The license must be GPL, since Con3D is a GPL library. If you want to use Simox under LGPL you must disable Coin3D support!") 
		SET (SIMOX_VISUALIZATION TRUE)
    	SET (SIMOX_VISUALIZATION_LIBS ${QT_LIBRARIES} ${COIN3D_LIBRARIES} ${SoQt_LIBRARIES} )
    	SET (SIMOX_VISUALIZATION_INCLUDE_PATHS ${SoQt_INCLUDE_DIRS} ${COIN3D_INCLUDE_DIRS} )
    	SET (SIMOX_VISUALIZATION_COMPILE_FLAGS "-DCOIN_DLL " "-DSOQT_DLL ")
	endif()
	
elseif (SIMOX_USE_OPENSCENEGRAPH_VISUALIZATION)

    MESSAGE(STATUS "Searching OSG and Qt...")

    FIND_PACKAGE(OpenSceneGraph REQUIRED osgViewer osgUtil osgDB osgGA)

	if (OPENSCENEGRAPH_FOUND)
	    MESSAGE (STATUS "Found OpenSceneGraph:" ${OPENSCENEGRAPH_INCLUDE_DIRS})
		##INCLUDE_DIRECTORIES(${OPENSCENEGRAPH_INCLUDE_DIRS})
	endif (OPENSCENEGRAPH_FOUND)
	
	if ( QT_FOUND )
		MESSAGE (STATUS "Found Qt4: " ${QT_INCLUDE_DIR})
		include(${QT_USE_FILE})
		#MESSAGE(STATUS "QT_LIBRARIES: " ${QT_LIBRARIES})
	else ( QT_FOUND )
		MESSAGE (STATUS "Did not found Qt. Disabling Qt/OSG support.")
	endif ( QT_FOUND )
	
	if (QT_FOUND AND OPENSCENEGRAPH_FOUND)
	    MESSAGE (STATUS "Enabling OSG/Qt support")
	    ### a little hack is needed here since osgQt is not supported in the FindOSG script
	    MESSAGE("OPENSCENEGRAPH_LIBRARIES: ${OPENSCENEGRAPH_LIBRARIES}")
	    LIST(GET OPENSCENEGRAPH_LIBRARIES 1 firstOsgLib)
	    MESSAGE("firstOsgLib: ${firstOsgLib}")
	    GET_FILENAME_COMPONENT(osgLibPath ${firstOsgLib} PATH)
	    MESSAGE("osgLibPath: ${osgLibPath}")
	    list(APPEND OPENSCENEGRAPH_LIBRARIES optimized)
	    list(APPEND OPENSCENEGRAPH_LIBRARIES ${osgLibPath}/osgQt.lib)
	    list(APPEND OPENSCENEGRAPH_LIBRARIES debug)
	    list(APPEND OPENSCENEGRAPH_LIBRARIES ${osgLibPath}/osgQtd.lib)
	    MESSAGE("OPENSCENEGRAPH_LIBRARIES: ${OPENSCENEGRAPH_LIBRARIES}")
		SET (SIMOX_VISUALIZATION TRUE)
    	SET (SIMOX_VISUALIZATION_LIBS ${QT_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} )
    	SET (SIMOX_VISUALIZATION_INCLUDE_PATHS ${OPENSCENEGRAPH_INCLUDE_DIRS} )
    	SET (SIMOX_VISUALIZATION_COMPILE_FLAGS "")
	endif()
	
else()
    MESSAGE(STATUS "Visualization disabled")
endif()
	
############################# SETUP PATHS #############################
IF(DEFINED SIMOX_DIR)
	get_filename_component(SIMOX_DIR ${SIMOX_DIR} ABSOLUTE)
ELSE()
	SET(SIMOX_DIR "${CurrentPath}/../..") # we assume a simox directory above VirtualRobot
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
