
# Setup paths, libs and external cmake files to be used for VirtualRobot


if (NOT (Simox_FIND_QUIETLY OR VirtualRobot_FIND_QUIETLY)) 
    MESSAGE(STATUS "SETTING LIBRARY DEPENDENCIES FOR VIRTUAL ROBOT")
    MESSAGE(STATUS " * VirtualRobot_VISUALIZATION: ${VirtualRobot_VISUALIZATION}")
endif()

IF (VirtualRobot_VISUALIZATION)
    # we need to check for Qt4
    IF(NOT "$ENV{QT_QMAKE_EXECUTABLE}" STREQUAL "")
        if (NOT (Simox_FIND_QUIETLY OR VirtualRobot_FIND_QUIETLY)) 
	        MESSAGE (STATUS "USING QT-PATH from environment variable QT_QMAKE_EXECUTABLE: $ENV{QT_QMAKE_EXECUTABLE}")
	    endif()
	    file(TO_CMAKE_PATH "$ENV{QT_QMAKE_EXECUTABLE}" QT_QMAKE_EXECUTABLE)
	ENDIF()
    FIND_PACKAGE(Qt4 4.6.0 COMPONENTS QtOpenGL QtCore QtGui)
ENDIF()

if (NOT (Simox_FIND_QUIETLY OR VirtualRobot_FIND_QUIETLY)) 
    MESSAGE(STATUS " * VirtualRobot_INCLUDE_DIRS: ${VirtualRobot_INCLUDE_DIRS}")
endif()
INCLUDE_DIRECTORIES(${VirtualRobot_INCLUDE_DIRS})

if (NOT (Simox_FIND_QUIETLY OR VirtualRobot_FIND_QUIETLY)) 
    MESSAGE(STATUS " * VirtualRobot_COMPILE_FLAGS: ${VirtualRobot_COMPILE_FLAGS}")
endif()
ADD_DEFINITIONS( ${VirtualRobot_COMPILE_FLAGS} )

if (NOT (Simox_FIND_QUIETLY OR VirtualRobot_FIND_QUIETLY)) 
    MESSAGE(STATUS " * VirtualRobot_LIBRARY_DIRS: ${VirtualRobot_LIBRARY_DIRS}")
endif()
LINK_DIRECTORIES( ${VirtualRobot_LIBRARY_DIRS} )

#MESSAGE(STATUS " * VirtualRobot_CMAKE_INCLUDE: ${VirtualRobot_CMAKE_INCLUDE}")
FOREACH(f ${VirtualRobot_CMAKE_INCLUDE})
    if (NOT (Simox_FIND_QUIETLY OR VirtualRobot_FIND_QUIETLY)) 
        MESSAGE(STATUS " * VirtualRobot_CMAKE_INCLUDE: ${f}")
    endif()
    INCLUDE(${f})
ENDFOREACH(f) 

