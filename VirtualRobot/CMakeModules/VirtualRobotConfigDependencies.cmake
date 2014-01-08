
    SET (VirtualRobot_EXTERNAL_INCLUDE_DIRS "")
    SET (VirtualRobot_EXTERNAL_LIBRARIES "")
    SET (VirtualRobot_EXTERNAL_LIBRARY_DIRS "")
    SET (VirtualRobot_EXTERNAL_LIBRARY_FLAGS "")
    SET (VirtualRobot_EXTERNAL_LIBRARY_CMAKE_INCLUDE "")

	############################# SETUP MODULES #############################
	MESSAGE (STATUS "module path: "  ${CMAKE_MODULE_PATH})

	#### Eigen
	FIND_PACKAGE (Eigen3 REQUIRED)
	if (Eigen3_FOUND)
	    SET (VirtualRobot_EXTERNAL_INCLUDE_DIRS ${VirtualRobot_EXTERNAL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIR})
	endif (Eigen3_FOUND)

	#### BOOST
	FIND_PACKAGE(Boost 1.42.0 COMPONENTS filesystem system unit_test_framework program_options thread REQUIRED)
	if (Boost_FOUND)
	    MESSAGE (STATUS "Boost found at: ${Boost_INCLUDE_DIR}")
	    SET (VirtualRobot_EXTERNAL_INCLUDE_DIRS ${VirtualRobot_EXTERNAL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
        SET (VirtualRobot_EXTERNAL_LIBRARY_DIRS ${VirtualRobot_EXTERNAL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
        SET (VirtualRobot_EXTERNAL_LIBRARIES ${VirtualRobot_EXTERNAL_LIBRARIES} ${Boost_LIBRARIES})
	    # disable boost auto linking
        SET (VirtualRobot_EXTERNAL_LIBRARY_FLAGS "${VirtualRobot_EXTERNAL_LIBRARY_FLAGS} -DBOOST_ALL_NO_LIB -DBOOST_PROGRAM_OPTIONS_DYN_LINK -DBOOST_FILESYSTEM_DYN_LINK -DBOOST_SYSTEM_DYN_LINK -DBOOST_UNIT_TEST_FRAMEWORK_DYN_LINK -DBOOST_THREAD_DYN_LINK")
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
	SET (VirtualRobot_VISUALIZATION FALSE)
	SET (VirtualRobot_VISUALIZATION_LIBS "")
	SET (VirtualRobot_VISUALIZATION_INCLUDE_PATHS "")
	SET (VirtualRobot_VISUALIZATION_COMPILE_FLAGS "")

	OPTION(VirtualRobot_USE_COIN_VISUALIZATION "Use Coin3D for visualization" ON)
	OPTION(VirtualRobot_USE_OPENSCENEGRAPH_VISUALIZATION "Use OpenSceneGraph for visualization" OFF)
	OPTION(VirtualRobot_USE_COLLADA "Enable the loading of robots from collada files" OFF)


	if (VirtualRobot_USE_COIN_VISUALIZATION)
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
			MESSAGE (STATUS "QT_USE_FILE: " ${QT_USE_FILE})
			include(${QT_USE_FILE})
			SET (VirtualRobot_EXTERNAL_LIBRARY_CMAKE_INCLUDE ${VirtualRobot_EXTERNAL_LIBRARY_CMAKE_INCLUDE} ${QT_USE_FILE})
			SET (VirtualRobot_EXTERNAL_LIBRARY_DIRS ${VirtualRobot_EXTERNAL_LIBRARY_DIRS} ${QT_LIBRARY_DIR})
			SET (VirtualRobot_EXTERNAL_LIBRARIES ${VirtualRobot_EXTERNAL_LIBRARIES} ${QT_LIBRARIES})

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
		    MESSAGE (STATUS "By using the Coin3D library, the license of Simox is not LGPL any more. The license must be GPL, since Coin3D is a GPL library. If you want to use Simox under LGPL you must disable Coin3D support!")
			SET (VirtualRobot_VISUALIZATION TRUE)
        	SET (VirtualRobot_VISUALIZATION_LIBS ${QT_LIBRARIES} ${COIN3D_LIBRARIES} ${SoQt_LIBRARIES} )
        	SET (VirtualRobot_VISUALIZATION_INCLUDE_PATHS ${QT_INCLUDE_DIR} ${SoQt_INCLUDE_DIRS} ${COIN3D_INCLUDE_DIRS} )
        	SET (VirtualRobot_VISUALIZATION_COMPILE_FLAGS " -DCOIN_DLL -DSOQT_DLL ")
		endif()

	elseif (VirtualRobot_USE_OPENSCENEGRAPH_VISUALIZATION)

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
		    if (UNIX)
			    list(APPEND OPENSCENEGRAPH_LIBRARIES ${osgLibPath}/libosgQt.so)
		    else()
			    list(APPEND OPENSCENEGRAPH_LIBRARIES optimized)
			    list(APPEND OPENSCENEGRAPH_LIBRARIES ${osgLibPath}/osgQt.lib)
		    	list(APPEND OPENSCENEGRAPH_LIBRARIES debug)
		    	list(APPEND OPENSCENEGRAPH_LIBRARIES ${osgLibPath}/osgQtd.lib)
		    endif()
		    MESSAGE("OPENSCENEGRAPH_LIBRARIES: ${OPENSCENEGRAPH_LIBRARIES}")
			SET (VirtualRobot_VISUALIZATION TRUE)
        	SET (VirtualRobot_VISUALIZATION_LIBS ${QT_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} )
        	SET (VirtualRobot_VISUALIZATION_INCLUDE_PATHS ${OPENSCENEGRAPH_INCLUDE_DIRS} )
        	SET (VirtualRobot_VISUALIZATION_COMPILE_FLAGS "")
		endif()

	else()
	    MESSAGE(STATUS "Visualization disabled")
	endif()

	if (VirtualRobot_USE_COLLADA)
	    MESSAGE(STATUS "Searching for Collada...")

    	FIND_PACKAGE(COLLADA_DOM REQUIRED 2.4)

    	IF(COLLADA_DOM_FOUND)
        	MESSAGE (STATUS "Found Collada")
        	MESSAGE (STATUS "Collada COLLADA_DOM_ROOT_DIR : ${COLLADA_DOM_ROOT_DIR}")
	        MESSAGE (STATUS "Collada Include DIRS: ${COLLADA_DOM_INCLUDE_DIRS}")
	        MESSAGE (STATUS "Collada Libs: ${COLLADA_DOM_LIBRARIES}")
	        MESSAGE (STATUS "Collada COLLADA_DOM_LIBRARY_DIRS: ${COLLADA_DOM_LIBRARY_DIRS}")

	        FIND_LIBRARY(COLLADA_LIBRARY ${COLLADA_DOM_LIBRARIES} ${COLLADA_DOM_LIBRARY_DIRS})
	        MESSAGE (STATUS "Collada Full Collada lib: ${COLLADA_LIBRARY}")

	        include_directories(${COLLADA_DOM_INCLUDE_DIRS})
	    ENDIF()
	endif()

    SET (VirtualRobot_EXTERNAL_INCLUDE_DIRS ${VirtualRobot_EXTERNAL_INCLUDE_DIRS} ${VirtualRobot_VISUALIZATION_INCLUDE_PATHS})
    SET (VirtualRobot_EXTERNAL_LIBRARIES ${VirtualRobot_EXTERNAL_LIBRARIES} ${VirtualRobot_VISUALIZATION_LIBS})
    SET (VirtualRobot_EXTERNAL_LIBRARY_FLAGS "${VirtualRobot_EXTERNAL_LIBRARY_FLAGS} ${VirtualRobot_VISUALIZATION_COMPILE_FLAGS}")
    MESSAGE(STATUS "VirtualRobot_EXTERNAL_INCLUDE_DIRS:${VirtualRobot_EXTERNAL_INCLUDE_DIRS}")
    MESSAGE(STATUS "VirtualRobot_EXTERNAL_LIBRARY_DIRS:${VirtualRobot_EXTERNAL_LIBRARY_DIRS}")
    MESSAGE(STATUS "VirtualRobot_EXTERNAL_LIBRARIES:${VirtualRobot_EXTERNAL_LIBRARIES}")
    MESSAGE(STATUS "VirtualRobot_EXTERNAL_LIBRARY_FLAGS:${VirtualRobot_EXTERNAL_LIBRARY_FLAGS}")
