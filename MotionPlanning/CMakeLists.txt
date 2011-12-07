PROJECT ( Saba )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)

MESSAGE (STATUS "\n ***** CONFIGURING Simox project Saba *****")
INCLUDE (config.cmake)

MESSAGE(STATUS "Saba: using VirtualRobotDir: ${SABA_VirtualRobotDir}")
MESSAGE("SIMOX_LIB_DIR: ${SIMOX_LIB_DIR}")
MESSAGE("SIMOX_BIN_DIR: ${SIMOX_BIN_DIR}")

SET(SOURCES
CSpace/CSpace.cpp
CSpace/CSpaceSampled.cpp
CSpace/CSpaceNode.cpp
CSpace/CSpacePath.cpp
CSpace/CSpaceTree.cpp
CSpace/Sampler.cpp
Planner/MotionPlanner.cpp
Planner/Rrt.cpp
Planner/BiRrt.cpp
Planner/GraspIkRrt.cpp
Visualization/RrtWorkspaceVisualization.cpp
PostProcessing/PathProcessor.cpp
PostProcessing/ShortcutProcessor.cpp
)

SET(INCLUDES
Saba.h
CSpace/CSpace.h
CSpace/CSpaceSampled.h
CSpace/CSpaceNode.h
CSpace/CSpacePath.h
CSpace/CSpaceTree.h
CSpace/Sampler.h
Planner/MotionPlanner.h
Planner/Rrt.h
Planner/BiRrt.h
Planner/GraspIkRrt.h
Visualization/RrtWorkspaceVisualization.h
PostProcessing/PathProcessor.h
PostProcessing/ShortcutProcessor.h
${SABA_SimoxDir}/VirtualRobot/definesVR.h
)


if (SIMOX_USE_COIN_VISUALIZATION)
	SET(SOURCES
	${SOURCES}
	Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.cpp
	)

	SET(INCLUDES
	${INCLUDES}
	Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h
	)
endif (SIMOX_USE_COIN_VISUALIZATION)


INCLUDE_DIRECTORIES(${SABA_VirtualRobotDir}/..)
INCLUDE_DIRECTORIES(${SABA_DIR})
LINK_DIRECTORIES (${SIMOX_LIB_DIR})

ADD_LIBRARY (Saba SHARED ${SOURCES} ${INCLUDES})
#MESSAGE("VIRTUAL_ROBOT_ROBOT_LINK_LIBRARIES:" ${VIRTUAL_ROBOT_LINK_LIBRARIES})
TARGET_LINK_LIBRARIES (Saba ${COLLISIONDETECTION_LIB} ${VIRTUAL_ROBOT_LINK_LIBRARIES} VirtualRobot)

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
