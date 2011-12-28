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
#ifndef _VirtualRobot_RobotNode_h_
#define _VirtualRobot_RobotNode_h_

#include "../VirtualRobotImportExport.h"

#include "../SceneObject.h"
#include "../RobotFactory.h"
#include "../CollisionDetection/CollisionModel.h"
#include "../Transformation/DHParameter.h"
#include "../Visualization/VisualizationNode.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/enable_shared_from_this.hpp>

#include <string>
#include <vector>


namespace VirtualRobot
{
class Robot;

/*!
	Each RobotNode owns three transformations:
	* preJointTransformation: This transformation is fixed.
	* jointTransformation: This is the flexible part of the joint. Here, the jointValue is used to compute the transformation according to the implementation of the joint type.
	* postJointTransformation: This is a fixed transformation that is applied after the jointTransformation.

	The visualization (of limb and/or coordinateAxis) is linked to the local coordinate sysytem of this joint: preJointTransformation*jointTransformation
	The global pose of this joint is computed by considering all transformations: preJointTransformation*jointTransformation*postJointTransformation


*/
class VIRTUAL_ROBOT_IMPORT_EXPORT RobotNode : public boost::enable_shared_from_this<RobotNode>, public SceneObject
{
public:
	friend class RobotFactory;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
	*/
	RobotNode(	RobotWeakPtr rob, 
				const std::string &name,
				const std::vector<std::string> &childrenNames,
				float jointLimitLo,
				float jointLimitHi,
				VisualizationNodePtr visualization = VisualizationNodePtr(), 
				CollisionModelPtr collisionModel = CollisionModelPtr(),
				float jointValueOffset = 0.0f,
				const SceneObject::Physics &p = SceneObject::Physics(),
				CollisionCheckerPtr colChecker = CollisionCheckerPtr());

	/*!
	*/
	virtual ~RobotNode();

	RobotPtr getRobot();

	/*!
		Set the joint value.
		\param q The joint value.
		\param updateTransformations When true, the transformation matrices of this joint and all child joints are updated (by calling applyJointValue()).
		\param clampToLimits Consider joint limits. When false an exception is thrown in case of invalid values.
	*/
	void setJointValue(float q, bool updateTransformations = true, bool clampToLimits = true);

	/*!
		Compute/Update the transformations of this joint and all child joints.
	*/
	void applyJointValue();
	/*!
		This method is only useful for root nodes (e.g. no parents are available). Then the pose of the robot can be set here.
	*/
	void applyJointValue(const Eigen::Matrix4f &globalPos);

	/// Checks if the given node is one of the own parents. 
	//bool isParent(RobotNodePtr parent){return false;};

	/*!
		All children and their children (and so on) are collected.
		The current instance is also added.
	*/
	void collectAllRobotNodes( std::vector< RobotNodePtr > &storeNodes);
	
	float getJointValue() const;

	/*!
		Checks if jointValue is within joint limits. If not jointValue is adjusted.
	*/
	void respectJointLimits(float &jointValue) const;

	/*!
		Checks if jointValue is within joint limits. If verbose is set a warning is printed.
	*/
	bool checkJointLimits( float jointValue, bool verbose = false ) const;

	/*!
		The postJoint transformation.
	*/
	inline Eigen::Matrix4f getPostJointTransformation() {return postJointTransformation;}

	/*!
		The preJoint transformation.
	*/
	inline Eigen::Matrix4f getPreJointTransformation() {return preJointTransformation;}
	
	/*!
		Initialize robot node. Here pointers to robot and children are created from names. 
		Be sure all children are created and registered to robot before calling initialize.
		Usually RobotFactory manages the initialization.
	*/
	virtual bool initialize(RobotNodePtr parent, bool initializeChildren = false);
	virtual void reset();

	/*!
		Calling this method will cause an exception, since RobotNodes are controlled via joint values.
	*/
	virtual void setGlobalPose( const Eigen::Matrix4f &pose );


	/*
		The global pose of a joint is not identical with the pose of it's visualization, 
		since the visualization is linked to the joint coordinate system but the complete transformation of the robot node must consider the post-joint transformation.
		So, the ScenObject's getGlobalPose method is overwritten in order to serve the global pose that everyone would expect.
	*/
	virtual Eigen::Matrix4f getGlobalPose() const {return globalPosePostJoint;}

	/*!
		The visualization is linked to the globalPose of this node.
		The end point of this node additionally considers the postJointTransformation (@see getGlobalPose()).
	*/
	virtual Eigen::Matrix4f getGlobalPoseVisualization() const {return globalPose;}

	/*!
		The joint of this robot node is located at globalPose.
		The end point of this node additionally considers the postJointTransformation (@see getGlobalPose()).
	*/
	virtual Eigen::Matrix4f getGlobalPoseJoint() const {return globalPose;}

	/*!
		Display the coordinate system of this RobotNode. This is the global pose of it's visualization with applying the postJoint transformation.
		\p enable Show or hide coordinate system
		\p scaling Size of coordinate system
		\p text Text to display at coordinate system. If not given, the name of this robot node will be displayed.
		\p visualizationType	This option is only needed when the current robot node does not yet own a visualization. 
								Then a visualziationNode has to be built and the \p visualizationType specifies which type of visualization should be used.
								If not given, the first registered visaulizationfactory is used.
	*/
	virtual void showCoordinateSystem( bool enable, float scaling = 1.0f, std::string *text = NULL, const std::string &visualizationType="");


	/*!
		Print status information.
	*/
	virtual void print(bool printChildren = false, bool printDecoration = true) const;

	virtual void addChildNode(RobotNodePtr child);

	/*
		Returns true when child is a children of this node. If recursive is true, all children of children etc are also queried.
	*/
	virtual bool hasChildNode(const RobotNodePtr child, bool recursive = false) const;
	/*
		Returns true when child is a children of this node. If recursive is true, all children of children etc are also queried.
	*/
	virtual bool hasChildNode(const std::string &child, bool recursive = false) const;


	float getJointLimitLo();
	float getJointLimitHi();

	/*!
		Set joint limits [rad]
	*/
	void setJointLimits(float lo, float hi);

	virtual bool isTranslationalJoint() const;
	virtual bool isRotationalJoint() const;


	/*!
		Visualize the structure of this RobotNode.
		\p enable Show or hide the structure visualization
		\p visualizationType	This option is only needed when the current robot node does not yet own a visualization. 
								Then a visualziationNode has to be build and the \p visualizationType specifies which type of visualization should be used.
								If not given, the first registered visaulizationfactory is used.
	*/
	virtual void showStructure( bool enable, const std::string &visualizationType="");


	/*!
		Find all robot nodes whose movements affect this RobtoNode
	*/
	std::vector<RobotNodePtr> getAllParents( RobotNodeSetPtr rns );
 
	//! Return parent node
	RobotNodePtr getParent();

	/*!
		Clone this RobotNode. 
		\param newRobot The newly created RobotNode belongs to newRobot.
		\param cloneChildren If true, all children are cloned (and their children, etc).
		\param initializeWithParent If given, the RobotNode is initialized with this parent.
		\param colChecker Must only be set if the cloned RobotNode should be registered to a different collision checker instance.
	*/
	virtual RobotNodePtr clone(RobotPtr newRobot, bool cloneChildren = true, RobotNodePtr initializeWithParent = RobotNodePtr(), CollisionCheckerPtr colChecker = CollisionCheckerPtr());

protected:

	///////////////////////// SETUP ////////////////////////////////////
	std::string parentName;
	std::vector<std::string> childrenNames;
	float jointValueOffset;
	float jointLimitLo,jointLimitHi;
	Eigen::Matrix4f preJointTransformation;
	Eigen::Matrix4f postJointTransformation;
	DHParameter optionalDHParameter;			// When the joint is defined via DH parameters they are stored here
	///////////////////////// SETUP ////////////////////////////////////

	virtual void updateTransformationMatrices();
	virtual void updateTransformationMatrices(const Eigen::Matrix4f &globalPose);
	float jointValue;							//< The joint value
	RobotWeakPtr robot;
	std::vector< RobotNodePtr > children;
	RobotNodePtr parent;
	Eigen::Matrix4f globalPosePostJoint;	//< The postJoint transformation applied to transformationJoint. Defines the starting pose for all child joints.

	/*!
	Derived classes must implement their clone method here.
	*/
	virtual RobotNodePtr _clone(const RobotPtr newRobot, const std::vector<std::string> newChildren, const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel) = 0;
};

} // namespace VirtualRobot

#endif // _VirtualRobot_RobotNode_h_
