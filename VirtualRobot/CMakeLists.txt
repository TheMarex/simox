PROJECT ( VirtualRobot )

MESSAGE (STATUS "\n ***** CONFIGURING Simox project VirtualRobot *****")

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)

INCLUDE(config.cmake)

MESSAGE (STATUS "VirtualRobot Binary directory: ${VR_BIN_DIR}")
MESSAGE (STATUS "VirtualRobot Library directory: ${VR_LIB_DIR}")

SET(SOURCES
CollisionDetection/CollisionChecker.cpp
CollisionDetection/CollisionModel.cpp
CollisionDetection/CDManager.cpp
EndEffector/EndEffector.cpp
EndEffector/EndEffectorActor.cpp
Nodes/RobotNode.cpp
Nodes/RobotNodePrismatic.cpp
Nodes/RobotNodePrismaticFactory.cpp
Nodes/RobotNodeRevolute.cpp
Nodes/RobotNodeRevoluteFactory.cpp
Nodes/RobotNodeFixed.cpp
Nodes/RobotNodeFixedFactory.cpp
Transformation/Transformation.cpp
Visualization/Visualization.cpp
Visualization/VisualizationNode.cpp
Visualization/ColorMap.cpp
Visualization/TriMeshModel.cpp
XML/BaseIO.cpp
XML/RobotIO.cpp
XML/SceneIO.cpp
XML/ObjectIO.cpp
IK/IKSolver.cpp
IK/DifferentialIK.cpp
IK/GenericIKSolver.cpp
IK/CoMIK.cpp
IK/PoseQualityMeasurement.cpp
IK/PoseQualityManipulability.cpp
MathTools.cpp
Robot.cpp
RobotConfig.cpp
RobotNodeSet.cpp
KinematicChain.cpp
RobotFactory.cpp
SceneObject.cpp
SceneObjectSet.cpp
Scene.cpp
Obstacle.cpp
VirtualRobotException.cpp
ReachabilitySpace.cpp
Grasp.cpp
GraspSet.cpp
ManipulationObject.cpp
BoundingBox.cpp
RuntimeEnvironment.cpp
CompressionRLE.cpp
LinkedCoordinate.cpp
)

SET(INCLUDES
CollisionDetection/CollisionChecker.h
CollisionDetection/CollisionModel.h
CollisionDetection/CDManager.h
CollisionDetection/CollisionModelImplementation.h
CollisionDetection/CollisionCheckerImplementation.h
EndEffector/EndEffector.h
EndEffector/EndEffectorActor.h
Nodes/RobotNode.h
Nodes/RobotNodeFactory.h
Nodes/RobotNodePrismatic.h
Nodes/RobotNodePrismaticFactory.h
Nodes/RobotNodeRevolute.h
Nodes/RobotNodeRevoluteFactory.h
Nodes/RobotNodeFixed.h
Nodes/RobotNodeFixedFactory.h
Transformation/DHParameter.h
Transformation/Transformation.h
Visualization/VisualizationFactory.h
Visualization/Visualization.h
Visualization/VisualizationNode.h
Visualization/ColorMap.h
Visualization/TriMeshModel.h
XML/BaseIO.h
XML/RobotIO.h
XML/SceneIO.h
XML/ObjectIO.h
IK/IKSolver.h
IK/DifferentialIK.h
IK/GenericIKSolver.h
IK/CoMIK.h
IK/PoseQualityMeasurement.h
IK/PoseQualityManipulability.h
AbstractFactoryMethod.h
VirtualRobot.h
MathTools.h
Robot.h
RobotConfig.h
RobotNodeSet.h
KinematicChain.h
RobotFactory.h
SceneObject.h
SceneObjectSet.h
Scene.h
Obstacle.h
VirtualRobotException.h
VirtualRobotImportExport.h
VirtualRobotTest.h
ReachabilitySpace.h
Grasp.h
GraspSet.h
ManipulationObject.h
BoundingBox.h
RuntimeEnvironment.h
CompressionRLE.h
)

ADD_SUBDIRECTORY(Visualization/tests)

if (SIMOX_VISUALIZATION AND SIMOX_USE_COIN_VISUALIZATION)

	SET(SOURCES
	${SOURCES}
	Visualization/CoinVisualization/CoinVisualizationFactory.cpp
	Visualization/CoinVisualization/CoinVisualizationNode.cpp
    Visualization/CoinVisualization/CoinVisualization.cpp
	)

	SET(INCLUDES
	${INCLUDES}
	Visualization/CoinVisualization/CoinVisualizationFactory.h
	Visualization/CoinVisualization/CoinVisualizationNode.h
    Visualization/CoinVisualization/CoinVisualization.h
	)
	ADD_SUBDIRECTORY(Visualization/CoinVisualization/tests)
	
elseif (SIMOX_VISUALIZATION AND SIMOX_USE_OPENSCENEGRAPH_VISUALIZATION)

	SET(SOURCES
	${SOURCES}
	Visualization/OSGVisualization/OSGVisualizationFactory.cpp
	Visualization/OSGVisualization/OSGVisualizationNode.cpp
    Visualization/OSGVisualization/OSGVisualization.cpp
    Visualization/OSGVisualization/osgQtWidget/osgViewerWidget.cpp
	)

	SET(INCLUDES
	${INCLUDES}
	Visualization/OSGVisualization/OSGVisualizationFactory.h
	Visualization/OSGVisualization/OSGVisualizationNode.h
    Visualization/OSGVisualization/OSGVisualization.h
    Visualization/OSGVisualization/osgQtWidget/osgViewerWidget.h
	)
	
	set(GUI_MOC_HDRS
        Visualization/OSGVisualization/osgQtWidget/osgViewerWidget.h
    )
MESSAGE("SOURCES:${SOURCES}")
    qt4_wrap_cpp(SOURCES ${GUI_MOC_HDRS})
MESSAGE("SOURCES:${SOURCES}")
    
	ADD_SUBDIRECTORY(Visualization/OSGVisualization/tests)
	#ADD_SUBDIRECTORY(Visualization/OSGVisualization/osgQtViewer)

endif ()

INCLUDE_DIRECTORIES(${SIMOX_VISUALIZATION_INCLUDE_PATHS})
ADD_DEFINITIONS(${SIMOX_VISUALIZATION_COMPILE_FLAGS})

# include collision detection
INCLUDE (CollisionDetection/CMakeLists.txt)

# include unit tests
ADD_SUBDIRECTORY(tests/)
ADD_SUBDIRECTORY(Transformation/tests/)

# include examples
ADD_SUBDIRECTORY(examples/)

# create configuration file which contains all compile time defines
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/definesVR.h.cmake ${PROJECT_SOURCE_DIR}/definesVR.h)


ADD_LIBRARY (VirtualRobot SHARED ${SOURCES} ${INCLUDES})
#MESSAGE("VIRTUAL_ROBOT_LINK_LIBRARIES:" ${VIRTUAL_ROBOT_LINK_LIBRARIES})
TARGET_LINK_LIBRARIES (VirtualRobot ${COLLISIONDETECTION_LIB} ${VIRTUAL_ROBOT_LINK_LIBRARIES})

# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${VR_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${VR_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${VR_LIB_DIR})

INSTALL(TARGETS VirtualRobot
        EXPORT VirtualRobotLibraryDepends
        DESTINATION ${VR_INSTALL_LIB_DIR})

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} DESTINATION ${VR_INSTALL_HEADER_DIR}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        PATTERN "CMakeModules" EXCLUDE
        PATTERN "tests" EXCLUDE
        PATTERN "build" EXCLUDE
        PATTERN "examples" EXCLUDE)
