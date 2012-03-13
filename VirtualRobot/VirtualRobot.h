/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_BasicDefinitions_h_
#define _VirtualRobot_BasicDefinitions_h_


/*! \defgroup VirtualRobot The VirtualRobot Library
* With the VirtualRobot library you can define complex robot structures, 
* perform collision detection, visualize robots and environments, do reachability analysis and generic IK solvers are provided.
*/



/** \mainpage Simox: A simulation, motion and grasp planning toolbox.
 
  \section Introduction Introduction
  
  The aim of the lightweight platform independent C++ toolbox \b Simox is to provide a set of
  algorithms for 3D simulation of robot systems, sampling based motion planning and grasp
  planning. Simox consists of three libraries (VirtualRobot, Saba and GraspStudio) and numerous 
  examples showing how these libraries can be used to build complex tools in the
  context of mobile manipulation. 
  
  \section VirtualRobot VirtualRobot
  
  The library \b VirtualRobot can be used to define complex
  robot systems, which may cover multiple robots with many degrees of freedom. The robot
  structure and its visualization can be easily defined via XML files and environments with
  obstacles and objects to manipulate are supported. Further, basic robot simulation components, 
  as Jacobian computations and generic Inverse Kinematics (IK) solvers, are offered by
  the library. Beyond that, extended features like tools for analyzing the reachable workspace
  for robotic manipulators or contact determination for grasping are included.
  \image html VR.png

  \section Saba Motion Planning
    
  With \b Saba, a library for planning collision-free motions is offered, which directly incorporates
  with the data provided by VirtualRobot. The algorithms cover state-of-the-art implementations 
  of sampling-based motion planning approaches (e.g. Rapidly-exploring Random Trees)
  and interfaces that allow to conveniently implement own planners. Since Saba was designed
  for planning in high-dimensional configuration spaces, complex planning problems for robots
  with a high number of degrees of freedom (DoF) can be solved efficiently.
  
  \image html Saba.png
  
  \section GraspStudio Grasp Planning
  
  \b GraspStudio offers possibilities to compute the grasp quality for generic end-effector definitions, 
  e.g. a humanoid hand. The implemented 6D wrench-space computations can be used
  to easily (and quickly) determine the quality of an applied grasp to an object. Furthermore,
  the implemented planners are able to generate grasp maps for given objects automatically.
  
  \image html GraspStudio.png
  
  \section Wiki Installation, tutorials and documentation

  Since complex frameworks have to incorporate with several libraries in order to provide full
  functionality, several issues may arise when setting up the environment, such as dependency
  problems, incompatible library versions or even non-existing ports of needed libraries for the
  used operating systems. Hence, only a limited set of libraries are used by the Simox core in
  order to make it compile. Extended functionality (e.g. visualization) can be turned off in
  order to allow Simox compiling on most platforms. Further dependencies are encapsulated
  with interfaces, making it easy to exchange e.g. the collision engine or the visualization
  functionality. As a reference implementation Simox offers Coin3D/SoQt-based visualization
  support.
    
  Please have a look at the wiki pages: http://sourceforge.net/apps/mediawiki/simox
 *
 */ 

// include compile time defines, generated by cmake
#include "definesVR.h"

#ifdef WIN32
// needed to have M_PI etc defined
#if !defined(_USE_MATH_DEFINES)
#define _USE_MATH_DEFINES
#endif

#endif


// allow std vector to be used with Eigen objects

#include<Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXf)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::MatrixXf)


#include <boost/shared_ptr.hpp>
#include <boost/assert.hpp>
#include <boost/weak_ptr.hpp>
#include <iostream>
#include <sstream>
#include <cmath>

namespace VirtualRobot
{
    // only valid within the VirtualRobot namespace
    using std::cout;
    using std::endl;

    class DHParameter;
    class RobotNode;
    class RobotNodeFactory;
    class RobotNodeSet;
    class KinematicChain;
    class Robot;
    class EndEffector;
    class EndEffectorActor;
    class CollisionChecker;
    class CollisionModel;
    class SceneObjectSet;
    class TriMeshModel;
    class SceneObject;
    class Obstacle;
	class Visualization;
    class VisualizationNode;
    class VisualizationFactory;
	class Scene;
	class RobotConfig;
	class Grasp;
	class GraspSet;
	class ManipulationObject;
	class CDManager;
	class Reachability;
	class WorkspaceRepresentation;
	class WorkspaceData;
	class PoseQualityMeasurement;
	class PoseQualityManipulability;

    typedef boost::shared_ptr<RobotNode> RobotNodePtr;
    typedef boost::shared_ptr<RobotNodeSet> RobotNodeSetPtr;
    typedef boost::shared_ptr<KinematicChain> KinematicChainPtr;
    typedef boost::weak_ptr<RobotNode> RobotNodeWeakPtr;
    typedef boost::shared_ptr<RobotNodeFactory> RobotNodeFactoryPtr;
    typedef boost::shared_ptr<Robot> RobotPtr;
    typedef boost::weak_ptr<Robot> RobotWeakPtr;
    typedef boost::shared_ptr<EndEffector> EndEffectorPtr;
    typedef boost::shared_ptr<EndEffectorActor> EndEffectorActorPtr;
    typedef boost::shared_ptr<CollisionModel> CollisionModelPtr;
    typedef boost::shared_ptr<CollisionChecker> CollisionCheckerPtr;
    typedef boost::shared_ptr<SceneObjectSet> SceneObjectSetPtr;
    typedef boost::shared_ptr<TriMeshModel> TriMeshModelPtr;
    typedef boost::shared_ptr<SceneObject> SceneObjectPtr;
    typedef boost::shared_ptr<Obstacle> ObstaclePtr;
    typedef boost::shared_ptr<Visualization> VisualizationPtr;
    typedef boost::shared_ptr<VisualizationNode> VisualizationNodePtr;
    typedef boost::shared_ptr<VisualizationFactory> VisualizationFactoryPtr;
    typedef boost::shared_ptr<WorkspaceData> WorkspaceDataPtr;
	typedef boost::shared_ptr<WorkspaceRepresentation> WorkspaceRepresentationPtr;
	typedef boost::shared_ptr<Reachability> ReachabilityPtr;
	typedef boost::shared_ptr<Scene> ScenePtr;
	typedef boost::shared_ptr<RobotConfig> RobotConfigPtr;
	typedef boost::shared_ptr<Grasp> GraspPtr;
	typedef boost::shared_ptr<GraspSet> GraspSetPtr;
	typedef boost::shared_ptr<ManipulationObject> ManipulationObjectPtr;
	typedef boost::shared_ptr<CDManager> CDManagerPtr;
	typedef boost::shared_ptr<PoseQualityMeasurement> PoseQualityMeasurementPtr;
	typedef boost::shared_ptr<PoseQualityManipulability> PoseQualityManipulabilityPtr;


#define VR_INFO std::cout <<__FILE__ << ":" << __LINE__ << ": "
#define VR_WARNING std::cerr <<__FILE__ << ":" << __LINE__ << " -Warning- "
#define VR_ERROR std::cerr <<__FILE__ << ":" << __LINE__ << " - ERROR - "

	
#ifdef _DEBUG
/*!
	This assert macro does nothing on RELEASE builds.
*/
#define VR_ASSERT( a )  BOOST_ASSERT( a )
	//THROW_VR_EXCEPTION_IF(!(a), "ASSERT failed (" << #a << ")" );

	// we have to switch to boost 1.48 to allow messages (BOOST_ASSERT_MSG) ....
#define VR_ASSERT_MESSAGE(a,b) BOOST_ASSERT(a)
	//THROW_VR_EXCEPTION_IF(!(a), "ASSERT failed (" << #a << "): " << b );
#else
#define VR_ASSERT(a)
#define VR_ASSERT_MESSAGE(a,b)
#endif

} // namespace

#endif // _VirtualRobot_RobotNode_h_
