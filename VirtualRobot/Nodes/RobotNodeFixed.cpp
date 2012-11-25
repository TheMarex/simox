
#include "RobotNodeFixed.h"
#include "../Robot.h"
#include "../Visualization//VisualizationNode.h"
#include "../CollisionDetection/CollisionModel.h"
#include <cmath>
#include <algorithm>
#include <boost/bind.hpp>
#include "../VirtualRobotException.h"

namespace VirtualRobot {

RobotNodeFixed::RobotNodeFixed(RobotWeakPtr rob, 
	const std::string &name,
	const Eigen::Matrix4f &preJointTransform,
	const Eigen::Matrix4f &postJointTransform,
	VisualizationNodePtr visualization, 
	CollisionModelPtr collisionModel,
	const SceneObject::Physics &p,
	CollisionCheckerPtr colChecker,
    RobotNodeType type
	) : RobotNode(rob,name,0.0f,0.0f,visualization,collisionModel,0.0f,p,colChecker,type)
{
	optionalDHParameter.isSet = false;
	this->preJointTransformation = preJointTransform;
	this->postJointTransformation = postJointTransform;
    checkValidRobotNodeType();
}

RobotNodeFixed::RobotNodeFixed(RobotWeakPtr rob, 
	const std::string &name,
	float a, float d, float alpha, float theta,
	VisualizationNodePtr visualization, 
	CollisionModelPtr collisionModel,
	const SceneObject::Physics &p,
	CollisionCheckerPtr colChecker,
    RobotNodeType type
	) : RobotNode(rob,name,0.0f,1.0f,visualization,collisionModel,0.0f,p,colChecker,type)
{
	initialized = false;
	optionalDHParameter.isSet = true;
	optionalDHParameter.setAInMM(a);
	optionalDHParameter.setDInMM(d);
	optionalDHParameter.setAlphaRadian(alpha, true);
	optionalDHParameter.setThetaRadian(theta, true);

	// compute DH transformation matrices
	Eigen::Matrix4f RotTheta = Eigen::Matrix4f::Identity();
	RotTheta(0,0) = cos(theta);
	RotTheta(0,1) = -sin(theta);
	RotTheta(1,0) = sin(theta);
	RotTheta(1,1) = cos(theta);
	Eigen::Matrix4f TransD = Eigen::Matrix4f::Identity();
	TransD(2,3) = d;
	Eigen::Matrix4f TransA = Eigen::Matrix4f::Identity();
	TransA(0,3) = a;
	Eigen::Matrix4f RotAlpha = Eigen::Matrix4f::Identity();
	RotAlpha(1,1) = cos(alpha);
	RotAlpha(1,2) = -sin(alpha);
	RotAlpha(2,1) = sin(alpha);
	RotAlpha(2,2) = cos(alpha);

	this->preJointTransformation = RotTheta;
	this->postJointTransformation = TransD*TransA*RotAlpha;
}

RobotNodeFixed::~RobotNodeFixed()
{
}

bool RobotNodeFixed::initialize(SceneObjectPtr parent, const std::vector<SceneObjectPtr> &children)
{
	return RobotNode::initialize(parent,children);
}

void RobotNodeFixed::updateTransformationMatrices(const Eigen::Matrix4f &parentPose)
{
	this->globalPose = parentPose * getPreJointTransformation();
	globalPosePostJoint = this->globalPose*getPostJointTransformation();
}

void RobotNodeFixed::print( bool printChildren, bool printDecoration ) const
{
	if (printDecoration)
		cout << "******** RobotNodeFixed ********" << endl;
	
	RobotNode::print(false,false);
	
	if (printDecoration)
		cout << "******** End RobotNodeFixed ********" << endl;


	std::vector< SceneObjectPtr > children = this->getChildren();
	if (printChildren)
		std::for_each(children.begin(), children.end(), boost::bind(&SceneObject::print, _1, true, true));
}


RobotNodePtr RobotNodeFixed::_clone(const RobotPtr newRobot, /*const std::vector<std::string> newChildren,*/ const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel, CollisionCheckerPtr colChecker)
{
	ReadLockPtr lock = getRobot()->getReadLock();
	
	RobotNodePtr result;

	if (optionalDHParameter.isSet)
		result.reset(new RobotNodeFixed(newRobot,name,optionalDHParameter.aMM(),optionalDHParameter.dMM(), optionalDHParameter.alphaRadian(), optionalDHParameter.thetaRadian(),visualizationModel,collisionModel,physics,colChecker,nodeType));
	else
		result.reset(new RobotNodeFixed(newRobot,name,getPreJointTransformation(),getPostJointTransformation(),visualizationModel,collisionModel,physics,colChecker,nodeType));

	return result;
}

void RobotNodeFixed::checkValidRobotNodeType()
{
    RobotNode::checkValidRobotNodeType();
    THROW_VR_EXCEPTION_IF (nodeType==Joint, "RobotNodeFixed not compatible with JointNode");
}


} // namespace
