
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
	const std::vector<std::string> &childrenNames,
	const Eigen::Matrix4f &preJointTransform,
	const Eigen::Matrix4f &postJointTransform,
	VisualizationNodePtr visualization, 
	CollisionModelPtr collisionModel,
	const SceneObject::Physics &p,
	CollisionCheckerPtr colChecker
	) : RobotNode(rob,name,childrenNames,0.0f,0.0f,visualization,collisionModel,0.0f,p,colChecker)
{
	optionalDHParameter.isSet = false;
	this->setPreJointTransformation(preJointTransform);
	this->setPostJointTransformation(postJointTransform);
}

RobotNodeFixed::RobotNodeFixed(RobotWeakPtr rob, 
	const std::string &name,
	const std::vector<std::string> &childrenNames,
	float a, float d, float alpha, float theta,
	VisualizationNodePtr visualization, 
	CollisionModelPtr collisionModel,
	const SceneObject::Physics &p,
	CollisionCheckerPtr colChecker
	) : RobotNode(rob,name,childrenNames,0.0f,1.0f,visualization,collisionModel,0.0f,p,colChecker)
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

	this->setPreJointTransformation(RotTheta);
	this->setPostJointTransformation(TransD*TransA*RotAlpha);
}

RobotNodeFixed::~RobotNodeFixed()
{
}

void RobotNodeFixed::reset()
{
	RobotNode::reset();
}

bool RobotNodeFixed::initialize(RobotNodePtr parent, bool initializeChildren)
{
	return RobotNode::initialize(parent,initializeChildren);
}

void RobotNodeFixed::updateTransformationMatrices()
{
	if (this->getParent())
		globalPose = this->getParent()->getGlobalPose() * getPreJointTransformation();
	else
		globalPose = getPreJointTransformation();

	globalPosePostJoint = globalPose*getPostJointTransformation();

	// update collision and visualization model
	SceneObject::setGlobalPose(globalPose);
}


void RobotNodeFixed::updateTransformationMatrices(const Eigen::Matrix4f &globalPose)
{
	THROW_VR_EXCEPTION_IF(this->getParent(),"This method could only be called on RobotNodes without parents.");

	this->globalPose = globalPose * getPreJointTransformation();

	globalPosePostJoint = this->globalPose*getPostJointTransformation();

	// update collision and visualization model
	SceneObject::setGlobalPose(this->globalPose);
}

void RobotNodeFixed::print( bool printChildren, bool printDecoration ) const
{
	if (printDecoration)
		cout << "******** RobotNodeFixed ********" << endl;
	
	RobotNode::print(false,false);
	
	if (printDecoration)
		cout << "******** End RobotNodeFixed ********" << endl;


	std::vector< RobotNodePtr > children = this->getChildren();
	if (printChildren)
		std::for_each(children.begin(), children.end(), boost::bind(&RobotNode::print, _1, true, true));
}


RobotNodePtr RobotNodeFixed::_clone(const RobotPtr newRobot, const std::vector<std::string> newChildren, const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel)
{
	RobotNodePtr result;

	if (optionalDHParameter.isSet)
		result.reset(new RobotNodeFixed(newRobot,name,newChildren,optionalDHParameter.aMM(),optionalDHParameter.dMM(), optionalDHParameter.alphaRadian(), optionalDHParameter.thetaRadian(),visualizationModel,collisionModel,physics));
	else
		result.reset(new RobotNodeFixed(newRobot,name,newChildren,getPreJointTransformation(),getPostJointTransformation(),visualizationModel,collisionModel,physics));

	return result;
}


} // namespace
