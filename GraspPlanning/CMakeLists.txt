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

if (Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)
	SET(SOURCES
	${SOURCES}
	Visualization/CoinVisualization/CoinConvexHullVisualization.cpp
	)

	SET(INCLUDES
	${INCLUDES}
	Visualization/CoinVisualization/CoinConvexHullVisualization.h
	)
endif ()

# this var is considered for generating Simox_INCLUDE_DIRS_
SET(GRASP_STUDIO_INCLUDE_DIRS "${GRASPSTUDIO_SimoxDir}" "${GRASPSTUDIO_DIR}" "${GRASPSTUDIO_DIR}/ExternalDependencies/qhull-2003.1/include")

INCLUDE_DIRECTORIES(${GRASPSTUDIO_SimoxDir})
INCLUDE_DIRECTORIES(${GRASPSTUDIO_DIR})
INCLUDE_DIRECTORIES(${GRASPSTUDIO_DIR}/ExternalDependencies/qhull-2003.1/include)

INCLUDE_DIRECTORIES(${VirtualRobot_EXTERNAL_INCLUDE_DIRS})
ADD_DEFINITIONS(${VirtualRobot_EXTERNAL_LIBRARY_FLAGS})


ADD_LIBRARY (GraspStudio SHARED ${SOURCES} ${INCLUDES})
#MESSAGE("VirtualRobot_ROBOT_LINK_LIBRARIES:" ${VirtualRobot_LINK_LIBRARIES})
TARGET_LINK_LIBRARIES (GraspStudio ${COLLISIONDETECTION_LIB} ${VirtualRobot_EXTERNAL_LIBRARIES} VirtualRobot qhull)

# include examples
ADD_SUBDIRECTORY(examples/)

# include unit tests
#ADD_SUBDIRECTORY(tests/)

IF (NOT DEFINED Simox_VERSION)
	set(GraspStudio_MAJOR_VERSION 1)
	set(GraspStudio_MINOR_VERSION 0)
	set(GraspStudio_PATCH_VERSION 0)
ELSE()
	set(GraspStudio_MAJOR_VERSION ${Simox_MAJOR_VERSION})
	set(GraspStudio_MINOR_VERSION ${Simox_MINOR_VERSION})
	set(GraspStudio_PATCH_VERSION ${Simox_PATCH_VERSION})
ENDIF()
set(GraspStudio_VERSION
    ${GraspStudio_MAJOR_VERSION}.${GraspStudio_MINOR_VERSION}.${GraspStudio_PATCH_VERSION})
set_target_properties( GraspStudio  PROPERTIES
	VERSION ${GraspStudio_VERSION}
	SOVERSION ${GraspStudio_MAJOR_VERSION}
	)
MESSAGE(STATUS " * GraspStudio Version: ${GraspStudio_VERSION}")


# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${GRASPSTUDIO_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${GRASPSTUDIO_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${GRASPSTUDIO_LIB_DIR})

INSTALL(TARGETS GraspStudio
  RUNTIME DESTINATION ${GRASPSTUDIO_INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${GRASPSTUDIO_INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${GRASPSTUDIO_INSTALL_LIB_DIR}
  COMPONENT Libraries
  )


INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} DESTINATION ${GRASPSTUDIO_INSTALL_HEADER_DIR}
	COMPONENT Headers
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        PATTERN "CMakeModules" EXCLUDE
        PATTERN "tests" EXCLUDE
        PATTERN "build*" EXCLUDE
        PATTERN "data" EXCLUDE
        PATTERN "examples" EXCLUDE
	)


