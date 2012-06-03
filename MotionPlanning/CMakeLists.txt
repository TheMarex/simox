PROJECT ( Saba )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)

MESSAGE (STATUS "\n ***** CONFIGURING Simox project Saba *****")
INCLUDE (config.cmake)

#MESSAGE(STATUS "Saba: using VirtualRobotDir: ${SABA_VirtualRobotDir}")
#MESSAGE(STATUS "      SIMOX_LIB_DIR: ${SIMOX_LIB_DIR}")
#MESSAGE(STATUS "      SIMOX_BIN_DIR: ${SIMOX_BIN_DIR}")
#MESSAGE ("SIMOX_USE_COIN_VISUALIZATION:${SIMOX_USE_COIN_VISUALIZATION}")

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

# this var is considered for generating SIMOX_INCLUDE_DIRS
SET(SABA_INCLUDE_DIRS "${SABA_DIR}")

INCLUDE_DIRECTORIES(${SABA_DIR})
LINK_DIRECTORIES (${SIMOX_LIB_DIR})

INCLUDE_DIRECTORIES(${VIRTUAL_ROBOT_EXTERNAL_INCLUDE_DIRS})
ADD_DEFINITIONS(${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_FLAGS})

ADD_LIBRARY (Saba SHARED ${SOURCES} ${INCLUDES})
#MESSAGE("VIRTUAL_ROBOT_ROBOT_LINK_LIBRARIES:" ${VIRTUAL_ROBOT_EXTERNAL_LIBRARIES})
TARGET_LINK_LIBRARIES (Saba VirtualRobot ${VIRTUAL_ROBOT_EXTERNAL_LIBRARIES})

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
  )

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} DESTINATION ${SABA_INSTALL_HEADER_DIR}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        PATTERN "CMakeModules" EXCLUDE
        PATTERN "tests" EXCLUDE
        PATTERN "build*" EXCLUDE
        PATTERN "data" EXCLUDE
        PATTERN "examples" EXCLUDE)
