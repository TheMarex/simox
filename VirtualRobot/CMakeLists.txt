PROJECT ( VirtualRobot )

MESSAGE (STATUS "\n ***** CONFIGURING Simox project VirtualRobot *****")

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)

INCLUDE(config.cmake)

MESSAGE (STATUS " * VirtualRobot Binary directory: ${VirtualRobot_BIN_DIR}")
MESSAGE (STATUS " * VirtualRobot Library directory: ${VirtualRobot_LIB_DIR}")

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
Nodes/RobotNodeActuator.cpp
Transformation/Transformation.cpp
Visualization/Visualization.cpp
Visualization/VisualizationNode.cpp
Visualization/ColorMap.cpp
Visualization/TriMeshModel.cpp
XML/BaseIO.cpp
XML/RobotIO.cpp
XML/SceneIO.cpp
XML/ObjectIO.cpp
XML/FileIO.cpp
IK/IKSolver.cpp
IK/DifferentialIK.cpp
IK/GenericIKSolver.cpp
IK/CoMIK.cpp
IK/PoseQualityMeasurement.cpp
IK/PoseQualityManipulability.cpp
IK/PoseQualityExtendedManipulability.cpp
Workspace/WorkspaceData.cpp
Workspace/WorkspaceRepresentation.cpp
Workspace/Reachability.cpp
Workspace/Manipulability.cpp
Workspace/WorkspaceGrid.cpp
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
Compression/CompressionRLE.cpp
Compression/CompressionBZip2.cpp
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
Nodes/RobotNodeActuator.h
Nodes/ConditionedLock.h
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
XML/FileIO.h
IK/IKSolver.h
IK/DifferentialIK.h
IK/GenericIKSolver.h
IK/CoMIK.h
IK/PoseQualityMeasurement.h
IK/PoseQualityManipulability.h
IK/PoseQualityExtendedManipulability.h
Workspace/WorkspaceData.h
Workspace/WorkspaceRepresentation.h
Workspace/Reachability.h
Workspace/Manipulability.h
Workspace/VoxelTree6D.hpp
Workspace/VoxelTree6DElement.hpp
Workspace/VoxelTreeND.hpp
Workspace/VoxelTreeNDElement.hpp
Workspace/WorkspaceGrid.h
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
Compression/CompressionRLE.h
Compression/CompressionBZip2.h
SphereApproximator.h
)

ADD_SUBDIRECTORY(Visualization/tests)
ADD_SUBDIRECTORY(Workspace/tests)
ADD_SUBDIRECTORY(Compression/tests)

if (VirtualRobot_VISUALIZATION AND VirtualRobot_USE_COIN_VISUALIZATION)

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
	
elseif (VirtualRobot_VISUALIZATION AND VirtualRobot_USE_OPENSCENEGRAPH_VISUALIZATION)

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

#INCLUDE_DIRECTORIES(${VirtualRobot_VISUALIZATION_INCLUDE_PATHS})
#ADD_DEFINITIONS(${VirtualRobot_VISUALIZATION_COMPILE_FLAGS})


#######################################################################################
# don't include -> this is meant for external libs
#INCLUDE(CMakeModules/VirtualRobotExternalLibrarySetup.cmake)

# manually apply the configuration of external libs
MESSAGE(STATUS " * VirtualRobot_EXTERNAL_INCLUDE_DIRS: ${VirtualRobot_EXTERNAL_INCLUDE_DIRS}")
INCLUDE_DIRECTORIES(${VirtualRobot_EXTERNAL_INCLUDE_DIRS})

MESSAGE(STATUS " * VirtualRobot_EXTERNAL_LIBRARY_FLAGS: ${VirtualRobot_EXTERNAL_LIBRARY_FLAGS}")
ADD_DEFINITIONS(${VirtualRobot_EXTERNAL_LIBRARY_FLAGS})

MESSAGE(STATUS " * VirtualRobot_EXTERNAL_LIBRARY_DIRS: ${VirtualRobot_EXTERNAL_LIBRARY_DIRS}")
LINK_DIRECTORIES( ${VirtualRobot_EXTERNAL_LIBRARY_DIRS} )

MESSAGE(STATUS " * VirtualRobot_EXTERNAL_LIBRARY_CMAKE_INCLUDE: ${VirtualRobot_EXTERNAL_LIBRARY_CMAKE_INCLUDE}")
FOREACH(f ${VirtualRobot_EXTERNAL_LIBRARY_CMAKE_INCLUDE})
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

MESSAGE (STATUS " * Setting standard data path to ${CMAKE_INSTALL_PREFIX}/data")
ADD_DEFINITIONS(-DVirtualRobot_DATA_PATH="${CMAKE_INSTALL_PREFIX}/data")
ADD_DEFINITIONS(-DVirtualRobot_SRC_DATA_PATH="${PROJECT_SOURCE_DIR}/data")

ADD_LIBRARY (VirtualRobot SHARED ${SOURCES} ${INCLUDES})

TARGET_LINK_LIBRARIES (VirtualRobot ${COLLISIONDETECTION_LIB} ${VirtualRobot_EXTERNAL_LIBRARIES})
# need this to let the dependencies show up in VirtualRobotLibraryDepends
TARGET_LINK_LIBRARIES (VirtualRobot LINK_INTERFACE_LIBRARIES ${COLLISIONDETECTION_LIB} ${VirtualRobot_EXTERNAL_LIBRARIES})

# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${VirtualRobot_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${VirtualRobot_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${VirtualRobot_LIB_DIR})





#######################################################################
# Setup for installation
#######################################################################

############################################
MESSAGE(STATUS " * Generating CMake files")

IF (NOT DEFINED Simox_VERSION)
	set(VirtualRobot_MAJOR_VERSION 2)
	set(VirtualRobot_MINOR_VERSION 1)
	set(VirtualRobot_PATCH_VERSION 9)
ELSE()
	set(VirtualRobot_MAJOR_VERSION ${Simox_MAJOR_VERSION})
	set(VirtualRobot_MINOR_VERSION ${Simox_MINOR_VERSION})
	set(VirtualRobot_PATCH_VERSION ${Simox_PATCH_VERSION})
ENDIF()
set(VirtualRobot_VERSION
    ${VirtualRobot_MAJOR_VERSION}.${VirtualRobot_MINOR_VERSION}.${VirtualRobot_PATCH_VERSION})
set_target_properties( VirtualRobot  PROPERTIES
	VERSION ${VirtualRobot_VERSION}
	SOVERSION ${VirtualRobot_MAJOR_VERSION}
	)
MESSAGE(STATUS " * VirtualRobot Version: ${VirtualRobot_VERSION}")

set(VirtualRobot_LIBRARIES VirtualRobot)
set(VirtualRobot_EXECUTABLES "")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
EXPORT(PACKAGE VirtualRobot)
 
#get_property(VirtualRobot_external_INCLUDE_DIRS TARGET VirtualRobot PROPERTY EXTERNAL_INCLUDE_DIRS)
#message(STATUS "VirtualRobot_external_INCLUDE_DIRS: ${VirtualRobot_external_INCLUDE_DIRS}")

SET (VirtualRobot_CONFIG_INBUILD_DIR ${VirtualRobot_BUILD_DIRECTORY}/share/VirtualRobot/cmake)
SET (VirtualRobot_CONFIG_INBUILD "${VirtualRobot_BUILD_DIRECTORY}/VirtualRobotConfig.cmake")
SET (VirtualRobot_CONFIG_INSTALL_LCOAL_DIR ${VirtualRobot_BUILD_DIRECTORY}/InstallFiles)
SET (VirtualRobot_CONFIG_INSTALL_LOCAL "${VirtualRobot_CONFIG_INSTALL_LCOAL_DIR}/VirtualRobotConfig.cmake")
SET (VirtualRobot_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/share/VirtualRobot/cmake)
SET (VirtualRobot_CONFIG_INSTALL "${CMAKE_INSTALL_PREFIX}/VirtualRobotConfig.cmake")

MESSAGE (STATUS " * VirtualRobot: SETTING UP INSTALL FILES")
MESSAGE (STATUS " ** In-build tree: VirtualRobot_CONFIG_INBUILD = ${VirtualRobot_CONFIG_INBUILD}")
MESSAGE (STATUS " ** Install tree : (local)   VirtualRobot_CONFIG_INSTALL_LOCAL = ${VirtualRobot_CONFIG_INSTALL_LOCAL}")
MESSAGE (STATUS " ** Install tree : (install) VirtualRobot_CONFIG_INSTALL = ${VirtualRobot_CONFIG_INSTALL}")

# Install the export set for use with the install-tree
install(
    EXPORT VirtualRobotLibraryDepends
    DESTINATION "${VirtualRobot_CONFIG_INSTALL_DIR}")


# Create an VirtualRobotConfig.cmake file for the use from the build tree
SET(VirtualRobot_LIBRARIES_ALL "${VirtualRobot_LIBRARIES}" "${VirtualRobot_EXTERNAL_LIBRARIES}")
SET(VirtualRobot_DIR "${PROJECT_SOURCE_DIR}/..")
SET(VirtualRobot_INCLUDE_DIRS "${VirtualRobot_DIR}" "${VirtualRobot_EXTERNAL_INCLUDE_DIRS}")
SET(VirtualRobot_LIB_DIRS "${VirtualRobot_BUILD_DIRECTORY}/${LIB_DIR}" "${VirtualRobot_EXTERNAL_LIBRARY_DIRS}")
SET(VirtualRobot_LIB_FLAGS "${VirtualRobot_EXTERNAL_LIBRARY_FLAGS}")
SET(VirtualRobot_CMAKE_DIR "${VirtualRobot_CONFIG_INBUILD_DIR}")
SET(VirtualRobot_LIBRARY_CMAKE_INCLUDE "${VirtualRobot_EXTERNAL_LIBRARY_CMAKE_INCLUDE}")
SET(VirtualRobot_DATA_PATH "${PROJECT_SOURCE_DIR}/data")
configure_file(
    ${VR_DIR}/CMakeModules/VirtualRobotConfig.cmake.in
    "${VirtualRobot_BUILD_DIRECTORY}/VirtualRobotConfig.cmake"
    @ONLY)
configure_file(
    ${VR_DIR}/CMakeModules/VirtualRobotConfigVersion.cmake.in
    "${VirtualRobot_BUILD_DIRECTORY}/VirtualRobotConfigVersion.cmake"
    @ONLY)
configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/VirtualRobotExternalLibrarySetup.cmake"
    "${VirtualRobot_CONFIG_INBUILD_DIR}/VirtualRobotExternalLibrarySetup.cmake" COPYONLY)
configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/VirtualRobotMacros.cmake"
    "${VirtualRobot_CONFIG_INBUILD_DIR}/VirtualRobotMacros.cmake" COPYONLY)
configure_file("${PROJECT_SOURCE_DIR}/CMakeModules/FindVirtualRobot.cmake"
    "${VirtualRobot_CONFIG_INBUILD_DIR}/FindVirtualRobot.cmake" COPYONLY)

    
    
# Create an VirtualRobotConfig.cmake file for the use from the install tree
# and install it
SET(VirtualRobot_LIBRARIES_ALL "${VirtualRobot_LIBRARIES}" "${VirtualRobot_EXTERNAL_LIBRARIES}")
SET(VirtualRobot_DIR "${CMAKE_INSTALL_PREFIX}/include")
SET(VirtualRobot_INCLUDE_DIRS "${VirtualRobot_DIR}" "${VirtualRobot_EXTERNAL_INCLUDE_DIRS}")
SET(VirtualRobot_LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${LIB_DIR}" "${VirtualRobot_EXTERNAL_LIBRARY_DIRS}")
SET(VirtualRobot_CMAKE_DIR "${VirtualRobot_CONFIG_INSTALL_DIR}")
SET(VirtualRobot_LIB_FLAGS "${VirtualRobot_EXTERNAL_LIBRARY_FLAGS}")
SET(VirtualRobot_LIBRARY_CMAKE_INCLUDE "${VirtualRobot_EXTERNAL_LIBRARY_CMAKE_INCLUDE}")
SET(VirtualRobot_DATA_PATH "${CMAKE_INSTALL_PREFIX}/data")

configure_file(
    ${VR_DIR}/CMakeModules/VirtualRobotConfig.cmake.in
    "${VirtualRobot_CONFIG_INSTALL_LCOAL_DIR}/VirtualRobotConfig.cmake"
    @ONLY)
configure_file(
    ${VR_DIR}/CMakeModules/VirtualRobotConfigVersion.cmake.in
    "${VirtualRobot_CONFIG_INSTALL_LCOAL_DIR}/VirtualRobotConfigVersion.cmake"
    @ONLY)

install(FILES
    "${VirtualRobot_CONFIG_INSTALL_LCOAL_DIR}/VirtualRobotConfig.cmake"
    "${VirtualRobot_CONFIG_INSTALL_LCOAL_DIR}/VirtualRobotConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_PREFIX}"
    COMPONENT ApplicationData
    )

install(FILES
    "${PROJECT_SOURCE_DIR}/CMakeModules/VirtualRobotExternalLibrarySetup.cmake"
    "${PROJECT_SOURCE_DIR}/CMakeModules/VirtualRobotMacros.cmake"
    "${PROJECT_SOURCE_DIR}/CMakeModules/FindVirtualRobot.cmake"
    DESTINATION "${VirtualRobot_CMAKE_DIR}"
    COMPONENT ApplicationData)


INSTALL(TARGETS VirtualRobot 
        EXPORT VirtualRobotLibraryDepends
        RUNTIME DESTINATION ${VirtualRobot_INSTALL_BIN_DIR}
        LIBRARY DESTINATION ${VirtualRobot_INSTALL_LIB_DIR}
        ARCHIVE DESTINATION ${VirtualRobot_INSTALL_LIB_DIR}
	COMPONENT Libraries
        )

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} DESTINATION ${VirtualRobot_INSTALL_HEADER_DIR}
	COMPONENT Headers
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        PATTERN "CMakeModules" EXCLUDE
        PATTERN "tests" EXCLUDE
        PATTERN "build*" EXCLUDE
        PATTERN "data" EXCLUDE
        PATTERN "examples" EXCLUDE
	)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/data DESTINATION ${CMAKE_INSTALL_PREFIX}
	COMPONENT ApplicationData
        FILES_MATCHING 
        PATTERN "*.xml"
        PATTERN "*.XML"
        PATTERN "*.iv"
        PATTERN "*.vrml"
        PATTERN "*.wrl"
        PATTERN "*.png"
        PATTERN "*.jpg"
        PATTERN "*.bin"
        PATTERN ".svn" EXCLUDE
	)

