PROJECT ( VirtualRobot )

MESSAGE (STATUS "\n ***** CONFIGURING Simox project VirtualRobot *****")

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)

INCLUDE(config.cmake)

MESSAGE (STATUS " * VirtualRobot Binary directory: ${VIRTUAL_ROBOT_BIN_DIR}")
MESSAGE (STATUS " * VirtualRobot Library directory: ${VIRTUAL_ROBOT_LIB_DIR}")

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
Workspace/WorkspaceData.cpp
Workspace/WorkspaceRepresentation.cpp
Workspace/Reachability.cpp
Grasping/Grasp.cpp
Grasping/GraspSet.cpp
Grasping/BasicGraspQualityMeasure.cpp
MathTools.cpp
Robot.cpp
RobotConfig.cpp
RobotNodeSet.cpp
Trajectory.cpp
KinematicChain.cpp
RobotFactory.cpp
SceneObject.cpp
SceneObjectSet.cpp
Scene.cpp
Obstacle.cpp
VirtualRobotException.cpp
ManipulationObject.cpp
BoundingBox.cpp
RuntimeEnvironment.cpp
CompressionRLE.cpp
LinkedCoordinate.cpp
SphereApproximator.cpp
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
Workspace/WorkspaceData.h
Workspace/WorkspaceRepresentation.h
Workspace/Reachability.h
Workspace/VoxelTree6D.hpp
Workspace/VoxelTree6DElement.hpp
Grasping/Grasp.h
Grasping/GraspSet.h
Grasping/BasicGraspQualityMeasure.h
AbstractFactoryMethod.h
VirtualRobot.h
MathTools.h
Robot.h
RobotConfig.h
RobotNodeSet.h
Trajectory.h
KinematicChain.h
RobotFactory.h
SceneObject.h
SceneObjectSet.h
Scene.h
Obstacle.h
VirtualRobotException.h
VirtualRobotImportExport.h
VirtualRobotTest.h
ManipulationObject.h
BoundingBox.h
RuntimeEnvironment.h
CompressionRLE.h
SphereApproximator.h
)

ADD_SUBDIRECTORY(Visualization/tests)
ADD_SUBDIRECTORY(Workspace/tests)

if (VIRTUAL_ROBOT_VISUALIZATION AND VIRTUAL_ROBOT_USE_COIN_VISUALIZATION)

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
	
elseif (VIRTUAL_ROBOT_VISUALIZATION AND VIRTUAL_ROBOT_USE_OPENSCENEGRAPH_VISUALIZATION)

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
    qt4_wrap_cpp(SOURCES ${GUI_MOC_HDRS})
    
	ADD_SUBDIRECTORY(Visualization/OSGVisualization/tests)
	#ADD_SUBDIRECTORY(Visualization/OSGVisualization/osgQtViewer)

endif ()

#INCLUDE_DIRECTORIES(${VIRTUAL_ROBOT_VISUALIZATION_INCLUDE_PATHS})
#ADD_DEFINITIONS(${VIRTUAL_ROBOT_VISUALIZATION_COMPILE_FLAGS})


#######################################################################################
# don't include -> this is meant for external libs
#INCLUDE(CMakeModules/VirtualRobotExternalLibrarySetup.cmake)

# manually apply the configuration of external libs
MESSAGE(STATUS " * VIRTUAL_ROBOT_EXTERNAL_INCLUDE_DIRS: ${VIRTUAL_ROBOT_EXTERNAL_INCLUDE_DIRS}")
INCLUDE_DIRECTORIES(${VIRTUAL_ROBOT_EXTERNAL_INCLUDE_DIRS})

MESSAGE(STATUS " * VIRTUAL_ROBOT_EXTERNAL_LIBRARY_FLAGS: ${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_FLAGS}")
ADD_DEFINITIONS(${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_FLAGS})

MESSAGE(STATUS " * VIRTUAL_ROBOT_EXTERNAL_LIBRARY_DIRS: ${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_DIRS}")
LINK_DIRECTORIES( ${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_DIRS} )

MESSAGE(STATUS " * VIRTUAL_ROBOT_EXTERNAL_LIBRARY_CMAKE_INCLUDE: ${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_CMAKE_INCLUDE}")
FOREACH(f ${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_CMAKE_INCLUDE})
    MESSAGE(STATUS " ** INCLUDE: ${f}")
    INCLUDE(${f})
ENDFOREACH(f) 
#######################################################################################

# include collision detection
INCLUDE (CollisionDetection/CMakeLists.txt)

# include unit tests
ADD_SUBDIRECTORY(tests/)
ADD_SUBDIRECTORY(Transformation/tests/)

# include examples
ADD_SUBDIRECTORY(examples/)

# create configuration file which contains all compile time defines
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/definesVR.h.cmake ${PROJECT_SOURCE_DIR}/definesVR.h)

ADD_DEFINITIONS(-DVIRTUAL_ROBOT_DATA_PATH="${CMAKE_INSTALL_PREFIX}/data")

ADD_LIBRARY (VirtualRobot SHARED ${SOURCES} ${INCLUDES})

TARGET_LINK_LIBRARIES (VirtualRobot ${COLLISIONDETECTION_LIB} ${VIRTUAL_ROBOT_EXTERNAL_LIBRARIES})
# need this to let the dependencies show up in VirtualRobotLibraryDepends
TARGET_LINK_LIBRARIES (VirtualRobot LINK_INTERFACE_LIBRARIES ${COLLISIONDETECTION_LIB} ${VIRTUAL_ROBOT_EXTERNAL_LIBRARIES})

# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${VIRTUAL_ROBOT_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${VIRTUAL_ROBOT_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${VIRTUAL_ROBOT_LIB_DIR})





#######################################################################
# Setup for installation
#######################################################################

############################################
MESSAGE(STATUS " * Generating CMake files")

set(VIRTUAL_ROBOT_MAJOR_VERSION 2)
set(VIRTUAL_ROBOT_MINOR_VERSION 1)
set(VIRTUAL_ROBOT_PATCH_VERSION 2)
set(VIRTUAL_ROBOT_VERSION
    ${VIRTUAL_ROBOT_MAJOR_VERSION}.${VIRTUAL_ROBOT_MINOR_VERSION}.${VIRTUAL_ROBOT_PATCH_VERSION})

set(VIRTUAL_ROBOT_LIBRARIES VirtualRobot)
set(VIRTUAL_ROBOT_EXECUTABLES "")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
EXPORT(PACKAGE VirtualRobot)
 

#get_property(VirtualRobot_external_INCLUDE_DIRS TARGET VirtualRobot PROPERTY EXTERNAL_INCLUDE_DIRS)
#message(STATUS "VirtualRobot_external_INCLUDE_DIRS: ${VirtualRobot_external_INCLUDE_DIRS}")

SET (VIRTUAL_ROBOT_CONFIG_INBUILD_DIR ${VIRTUAL_ROBOT_BUILD_DIRECTORY}/share/VirtualRobot/cmake)
SET (VIRTUAL_ROBOT_CONFIG_INBUILD "${VIRTUAL_ROBOT_BUILD_DIRECTORY}/VirtualRobotConfig.cmake")
SET (VIRTUAL_ROBOT_CONFIG_INSTALL_LCOAL_DIR ${VIRTUAL_ROBOT_BUILD_DIRECTORY}/InstallFiles)
SET (VIRTUAL_ROBOT_CONFIG_INSTALL_LOCAL "${VIRTUAL_ROBOT_CONFIG_INSTALL_LCOAL_DIR}/VirtualRobotConfig.cmake")
SET (VIRTUAL_ROBOT_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/share/VirtualRobot/cmake)
SET (VIRTUAL_ROBOT_CONFIG_INSTALL "${CMAKE_INSTALL_PREFIX}/VirtualRobotConfig.cmake")

MESSAGE (STATUS " * VirtualRobot: SETTING UP INSTALL FILES")
MESSAGE (STATUS " ** In-build tree: VIRTUAL_ROBOT_CONFIG_INBUILD = ${VIRTUAL_ROBOT_CONFIG_INBUILD}")
MESSAGE (STATUS " ** Install tree : (local)   VIRTUAL_ROBOT_CONFIG_INSTALL_LOCAL = ${VIRTUAL_ROBOT_CONFIG_INSTALL_LOCAL}")
MESSAGE (STATUS " ** Install tree : (install) VIRTUAL_ROBOT_CONFIG_INSTALL = ${VIRTUAL_ROBOT_CONFIG_INSTALL}")

# Install the export set for use with the install-tree
install(
    EXPORT VirtualRobotLibraryDepends
    DESTINATION "${VIRTUAL_ROBOT_CONFIG_INSTALL_DIR}")


# Create an VirtualRobotConfig.cmake file for the use from the build tree
SET(VIRTUAL_ROBOT_LIBRARIES_ALL "${VIRTUAL_ROBOT_LIBRARIES}" "${VIRTUAL_ROBOT_EXTERNAL_LIBRARIES}")
SET(VIRTUAL_ROBOT_DIR "${PROJECT_SOURCE_DIR}/..")
SET(VIRTUAL_ROBOT_INCLUDE_DIRS "${VIRTUAL_ROBOT_DIR}" "${VIRTUAL_ROBOT_EXTERNAL_INCLUDE_DIRS}")
SET(VIRTUAL_ROBOT_LIB_DIRS "${VIRTUAL_ROBOT_BUILD_DIRECTORY}/${LIB_DIR}" "${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_DIRS}")
SET(VIRTUAL_ROBOT_LIB_FLAGS "${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_FLAGS}")
SET(VIRTUAL_ROBOT_CMAKE_DIR "${VIRTUAL_ROBOT_CONFIG_INBUILD_DIR}")
SET(VIRTUAL_ROBOT_LIBRARY_CMAKE_INCLUDE "${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_CMAKE_INCLUDE}")
SET(VIRTUAL_ROBOT_DATA_PATH "${PROJECT_SOURCE_DIR}/data")
configure_file(
    ${VR_DIR}/CMakeModules/VirtualRobotConfig.cmake.in
    "${VIRTUAL_ROBOT_BUILD_DIRECTORY}/VirtualRobotConfig.cmake"
    @ONLY)
configure_file(
    ${VR_DIR}/CMakeModules/VirtualRobotConfigVersion.cmake.in
    "${VIRTUAL_ROBOT_BUILD_DIRECTORY}/VirtualRobotConfigVersion.cmake"
    @ONLY)
configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/VirtualRobotExternalLibrarySetup.cmake"
    "${VIRTUAL_ROBOT_CONFIG_INBUILD_DIR}/VirtualRobotExternalLibrarySetup.cmake" COPYONLY)
configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/VirtualRobotMacros.cmake"
    "${VIRTUAL_ROBOT_CONFIG_INBUILD_DIR}/VirtualRobotMacros.cmake" COPYONLY)
configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/FindVirtualRobot.cmake"
    "${VIRTUAL_ROBOT_CONFIG_INBUILD_DIR}/FindVirtualRobot.cmake" COPYONLY)

    
    
# Create an VirtualRobotConfig.cmake file for the use from the install tree
# and install it
SET(VIRTUAL_ROBOT_LIBRARIES_ALL "${VIRTUAL_ROBOT_LIBRARIES}" "${VIRTUAL_ROBOT_EXTERNAL_LIBRARIES}")
SET(VIRTUAL_ROBOT_DIR "${CMAKE_INSTALL_PREFIX}/include")
SET(VIRTUAL_ROBOT_INCLUDE_DIRS "${VIRTUAL_ROBOT_DIR}" "${VIRTUAL_ROBOT_EXTERNAL_INCLUDE_DIRS}")
SET(VIRTUAL_ROBOT_LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${LIB_DIR}" "${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_DIRS}")
SET(VIRTUAL_ROBOT_CMAKE_DIR "${VIRTUAL_ROBOT_CONFIG_INSTALL_DIR}")
SET(VIRTUAL_ROBOT_LIB_FLAGS "${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_FLAGS}")
SET(VIRTUAL_ROBOT_LIBRARY_CMAKE_INCLUDE "${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_CMAKE_INCLUDE}")
SET(VIRTUAL_ROBOT_DATA_PATH "${CMAKE_INSTALL_PREFIX}/data")

configure_file(
    ${VR_DIR}/CMakeModules/VirtualRobotConfig.cmake.in
    "${VIRTUAL_ROBOT_CONFIG_INSTALL_LCOAL_DIR}/VirtualRobotConfig.cmake"
    @ONLY)
configure_file(
    ${VR_DIR}/CMakeModules/VirtualRobotConfigVersion.cmake.in
    "${VIRTUAL_ROBOT_CONFIG_INSTALL_LCOAL_DIR}/VirtualRobotConfigVersion.cmake"
    @ONLY)
install(FILES
    "${VIRTUAL_ROBOT_CONFIG_INSTALL_LCOAL_DIR}/VirtualRobotConfig.cmake"
    "${VIRTUAL_ROBOT_CONFIG_INSTALL_LCOAL_DIR}/VirtualRobotConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
install(FILES
    "${PROJECT_SOURCE_DIR}/CMakeModules/VirtualRobotExternalLibrarySetup.cmake"
    "${PROJECT_SOURCE_DIR}/CMakeModules/VirtualRobotMacros.cmake"
    "${PROJECT_SOURCE_DIR}/CMakeModules/FindVirtualRobot.cmake"
    DESTINATION "${VIRTUAL_ROBOT_CMAKE_DIR}")


INSTALL(TARGETS VirtualRobot 
        EXPORT VirtualRobotLibraryDepends
        RUNTIME DESTINATION ${VIRTUAL_ROBOT_INSTALL_BIN_DIR}
        LIBRARY DESTINATION ${VIRTUAL_ROBOT_INSTALL_LIB_DIR}
        ARCHIVE DESTINATION ${VIRTUAL_ROBOT_INSTALL_LIB_DIR}
        )

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} DESTINATION ${VIRTUAL_ROBOT_INSTALL_HEADER_DIR}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        PATTERN "CMakeModules" EXCLUDE
        PATTERN "tests" EXCLUDE
        PATTERN "build*" EXCLUDE
        PATTERN "data" EXCLUDE
        PATTERN "examples" EXCLUDE)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/data DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING 
        PATTERN "*.xml"
        PATTERN "*.XML"
        PATTERN "*.iv"
        PATTERN "*.vrml"
        PATTERN "*.wrl"
        PATTERN "*.png"
        PATTERN "*.jpg"
        PATTERN "*.bin"
        PATTERN ".svn" EXCLUDE)
