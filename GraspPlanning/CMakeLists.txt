PROJECT ( GraspStudio )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)

MESSAGE (STATUS "\n ***** CONFIGURING Simox project GraspStudio *****")
INCLUDE (config.cmake)

########### QHULL #################
OPTION(BUILD_qhull "if on builds qhull external library" ON)
IF(BUILD_qhull)
	ADD_SUBDIRECTORY(ExternalDependencies/qhull-2003.1)
ENDIF(BUILD_qhull)
########### END QHULL #################

SET(SOURCES
ConvexHullGenerator.cpp
ContactConeGenerator.cpp
ApproachMovementGenerator.cpp
ApproachMovementSurfaceNormal.cpp
GraspPlanner/GraspPlanner.cpp
GraspPlanner/GenericGraspPlanner.cpp
GraspQuality/GraspQualityMeasure.cpp
GraspQuality/GraspQualityMeasureWrenchSpace.cpp
Visualization/ConvexHullVisualization.cpp
)

SET(INCLUDES
GraspStudio.h
ConvexHullGenerator.h
ContactConeGenerator.h
ApproachMovementGenerator.h
ApproachMovementSurfaceNormal.h
GraspPlanner/GraspPlanner.h
GraspPlanner/GenericGraspPlanner.h
GraspQuality/GraspQualityMeasure.h
GraspQuality/GraspQualityMeasureWrenchSpace.h
Visualization/ConvexHullVisualization.h
${GRASPSTUDIO_SimoxDir}/VirtualRobot/definesVR.h
)

if (SIMOX_USE_COIN_VISUALIZATION)
	SET(SOURCES
	${SOURCES}
	Visualization/CoinVisualization/CoinConvexHullVisualization.cpp
	)

	SET(INCLUDES
	${INCLUDES}
	Visualization/CoinVisualization/CoinConvexHullVisualization.h
	)
endif (SIMOX_USE_COIN_VISUALIZATION)


INCLUDE_DIRECTORIES(${GRASPSTUDIO_VirtualRobotDir}/..)
INCLUDE_DIRECTORIES(${GRASPSTUDIO_DIR})
INCLUDE_DIRECTORIES(${GRASPSTUDIO_DIR}/ExternalDependencies/qhull-2003.1/include)
LINK_DIRECTORIES (${VR_LIB_DIR})

ADD_LIBRARY (GraspStudio SHARED ${SOURCES} ${INCLUDES})
#MESSAGE("VIRTUAL_ROBOT_ROBOT_LINK_LIBRARIES:" ${VIRTUAL_ROBOT_LINK_LIBRARIES})
TARGET_LINK_LIBRARIES (GraspStudio ${COLLISIONDETECTION_LIB} ${VIRTUAL_ROBOT_LINK_LIBRARIES} VirtualRobot qhull)

# include examples
ADD_SUBDIRECTORY(examples/)

# include unit tests
#ADD_SUBDIRECTORY(tests/)


# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${GRASPSTUDIO_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${GRASPSTUDIO_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${GRASPSTUDIO_LIB_DIR})
