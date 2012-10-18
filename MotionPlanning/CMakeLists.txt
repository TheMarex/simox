PROJECT ( Saba )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)

MESSAGE (STATUS "\n ***** CONFIGURING Simox project Saba *****")
INCLUDE (config.cmake)

#MESSAGE(STATUS "Saba: using VirtualRobotDir: ${SABA_VirtualRobotDir}")
#MESSAGE(STATUS "      Simox_LIB_DIR: ${Simox_LIB_DIR}")
#MESSAGE(STATUS "      Simox_BIN_DIR: ${Simox_BIN_DIR}")
#MESSAGE ("Simox_USE_COIN_VISUALIZATION:${Simox_USE_COIN_VISUALIZATION}")


SET(SOURCES
CSpace/CSpace.cpp
CSpace/CSpaceSampled.cpp
CSpace/CSpaceNode.cpp
CSpace/CSpacePath.cpp
CSpace/CSpaceTree.cpp
CSpace/Sampler.cpp
CSpace/ConfigurationConstraint.cpp
Planner/MotionPlanner.cpp
Planner/Rrt.cpp
Planner/BiRrt.cpp
Planner/GraspIkRrt.cpp
Planner/GraspRrt.cpp
Planner/PlanningThread.cpp
Visualization/RrtWorkspaceVisualization.cpp
PostProcessing/PathProcessor.cpp
PostProcessing/ShortcutProcessor.cpp
PostProcessing/PathProcessingThread.cpp
ApproachDiscretization.cpp
)

SET(INCLUDES
Saba.h
CSpace/CSpace.h
CSpace/CSpaceSampled.h
CSpace/CSpaceNode.h
CSpace/CSpacePath.h
CSpace/CSpaceTree.h
CSpace/Sampler.h
CSpace/ConfigurationConstraint.h
Planner/MotionPlanner.h
Planner/Rrt.h
Planner/BiRrt.h
Planner/GraspIkRrt.h
Planner/GraspRrt.h
Planner/PlanningThread.h
Visualization/RrtWorkspaceVisualization.h
PostProcessing/PathProcessor.h
PostProcessing/ShortcutProcessor.h
PostProcessing/PathProcessingThread.h
ApproachDiscretization.h
${SABA_SimoxDir}/VirtualRobot/definesVR.h
)


if (Simox_USE_COIN_VISUALIZATION)
	SET(SOURCES
	${SOURCES}
	Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.cpp
	)

	SET(INCLUDES
	${INCLUDES}
	Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h
	)
endif (Simox_USE_COIN_VISUALIZATION)

# this var is considered for generating Simox_INCLUDE_DIRS
SET(SABA_INCLUDE_DIRS "${SABA_DIR}")

INCLUDE_DIRECTORIES(${SABA_DIR})
LINK_DIRECTORIES (${Simox_LIB_DIR})

INCLUDE_DIRECTORIES(${VirtualRobot_EXTERNAL_INCLUDE_DIRS})
ADD_DEFINITIONS(${VirtualRobot_EXTERNAL_LIBRARY_FLAGS})

ADD_LIBRARY (Saba SHARED ${SOURCES} ${INCLUDES})
#MESSAGE("VirtualRobot_ROBOT_LINK_LIBRARIES:" ${VirtualRobot_EXTERNAL_LIBRARIES})
TARGET_LINK_LIBRARIES (Saba VirtualRobot ${VirtualRobot_EXTERNAL_LIBRARIES})

IF (NOT DEFINED Simox_VERSION)
	set(Saba_MAJOR_VERSION 2)
	set(Saba_MINOR_VERSION 1)
	set(Saba_PATCH_VERSION 5)
ELSE()
	set(Saba_MAJOR_VERSION ${Simox_MAJOR_VERSION})
	set(Saba_MINOR_VERSION ${Simox_MINOR_VERSION})
	set(Saba_PATCH_VERSION ${Simox_PATCH_VERSION})
ENDIF()
set(Saba_VERSION
    ${Saba_MAJOR_VERSION}.${Saba_MINOR_VERSION}.${Saba_PATCH_VERSION})
set_target_properties( Saba  PROPERTIES
	VERSION ${Saba_VERSION}
	SOVERSION ${Saba_MAJOR_VERSION}
	)
MESSAGE(STATUS " * Saba Version: ${Saba_VERSION}")


# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${SABA_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${SABA_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${SABA_LIB_DIR})

# include examples
ADD_SUBDIRECTORY(examples/)

# include unit tests
ADD_SUBDIRECTORY(tests/)

INSTALL(TARGETS Saba
  RUNTIME DESTINATION ${SABA_INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${SABA_INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${SABA_INSTALL_LIB_DIR}
  COMPONENT Libraries
  )

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} DESTINATION ${SABA_INSTALL_HEADER_DIR}
	COMPONENT Headers
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        PATTERN "CMakeModules" EXCLUDE
        PATTERN "tests" EXCLUDE
        PATTERN "build*" EXCLUDE
        PATTERN "data" EXCLUDE
        PATTERN "examples" EXCLUDE
	)

