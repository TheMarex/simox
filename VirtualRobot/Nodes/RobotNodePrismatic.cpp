
#include "RobotNodePrismatic.h"
#include "../Robot.h"
#include <cmath>
#include <algorithm>
#include <boost/bind.hpp>
#include <Eigen/Geometry> 
#include "../VirtualRobotException.h"

namespace VirtualRobot {

RobotNodePrismatic::RobotNodePrismatic(RobotWeakPtr rob, 
	const std::string &name,
	const std::vector<std::string> &childrenNames,
	float jointLimitLo,
	float jointLimitHi,
	const Eigen::Matrix4f &preJointTransform,
	const Eigen::Vector3f &translationDirection, 
	const Eigen::Matrix4f &postJointTransform,
	VisualizationNodePtr visualization, 
	CollisionModelPtr collisionModel,
	float jointValueOffset,
	const SceneObject::Physics &p,
	CollisionCheckerPtr colChecker
	) : RobotNode(rob,name,childrenNames,jointLimitLo,jointLimitHi,visualization,collisionModel,jointValueOffset,p,colChecker)

{
	initialized = false;
	optionalDHParameter.isSet = false;
	this->preJointTransformation = preJointTransform;
	this->jointTranslationDirection = translationDirection;
	this->postJointTransformation = postJointTransform;
}

RobotNodePrismatic::RobotNodePrismatic(RobotWeakPtr rob, 
	const std::string &name,
	const std::vector<std::string> &childrenNames,
	float jointLimitLo,
	float jointLimitHi,
	float a, float d, float alpha, float theta,
	VisualizationNodePtr visualization, 
	CollisionModelPtr collisionModel,
	float jointValueOffset,
	const SceneObject::Physics &p,
	CollisionCheckerPtr colChecker
	) : RobotNode(rob,name,childrenNames,jointLimitLo,jointLimitHi,visualization,collisionModel,jointValueOffset,p,colChecker)

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

	// fixed rotation around theta
	preJointTransformation = RotTheta;
	// joint setup
	jointTranslationDirection = Eigen::Vector3f(0,0,1);	// translation along the z axis
	// compute postJointTransformation
	postJointTransformation = TransD*TransA*RotAlpha;
}

RobotNodePrismatic::~RobotNodePrismatic()
{
}

void RobotNodePrismatic::reset()
{
	jointTranslationDirection = Eigen::Vector3f(0,0,1);
	RobotNode::reset();
}

bool RobotNodePrismatic::initialize(RobotNodePtr parent, bool initializeChildren)
{
	return RobotNode::initialize(parent,initializeChildren);
}

void RobotNodePrismatic::updateTransformationMatrices()
{
	if (this->getParent())
		globalPose = this->getParent()->getGlobalPose() * preJointTransformation;
	else
		globalPose = preJointTransformation;

	Eigen::Affine3f tmpT(Eigen::Translation3f((this->getJointValue()+jointValueOffset)*jointTranslationDirection));
	globalPose *= tmpT.matrix();

	globalPosePostJoint = globalPose*postJointTransformation;

	// update collision and visualization model
	// here we do not consider the postJointTransformation, since it already defines the transformation to the next joint.
	SceneObject::setGlobalPose(globalPose);
}

void RobotNodePrismatic::updateTransformationMatrices(const Eigen::Matrix4f &globalPose)
{
	THROW_VR_EXCEPTION_IF(this->getParent(),"This method could only be called on RobotNodes without parents.");

	this->globalPose = globalPose * preJointTransformation;

	Eigen::Affine3f tmpT(Eigen::Translation3f((this->getJointValue()+jointValueOffset)*jointTranslationDirection));
	this->globalPose *= tmpT.matrix();

	globalPosePostJoint = this->globalPose*postJointTransformation;

	// update collision and visualization model
	// here we do not consider the postJointTransformation, since it already defines the transformation to the next joint.
	SceneObject::setGlobalPose(this->globalPose);
}

void RobotNodePrismatic::print( bool printChildren, bool printDecoration ) const
{
	if (printDecoration)
		cout << "******** RobotNodePrismatic ********" << endl;

	RobotNode::print(false,false);

	cout << "* jointTranslationDirection: " << jointTranslationDirection[0] << ", " << jointTranslationDirection[1] << "," << jointTranslationDirection[2] << endl;

	if (printDecoration)
		cout << "******** End RobotNodePrismatic ********" << endl;
	

	std::vector< RobotNodePtr > children = this->getChildren();
	if (printChildren)
		std::for_each(children.begin(), children.end(), boost::bind(&RobotNode::print, _1, true, true));
}

RobotNodePtr RobotNodePrismatic::_clone(const RobotPtr newRobot, const std::vector<std::string> newChildren, const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel)
{
	RobotNodePtr result;

	if (optionalDHParameter.isSet)
		result.reset(new RobotNodePrismatic(newRobot,name,newChildren, jointLimitLo,jointLimitHi,optionalDHParameter.aMM(),optionalDHParameter.dMM(), optionalDHParameter.alphaRadian(), optionalDHParameter.thetaRadian(),visualizationModel,collisionModel, jointValueOffset,physics));
	else
		result.reset(new RobotNodePrismatic(newRobot,name,newChildren,jointLimitLo,jointLimitHi,preJointTransformation,jointTranslationDirection,postJointTransformation,visualizationModel,collisionModel,jointValueOffset,physics));
	return result;
}

bool RobotNodePrismatic::isTranslationalJoint() const
{
	return true;
}


Eigen::Vector3f RobotNodePrismatic::getJointTranslationDirection(const SceneObjectPtr coordSystem) const
{
	Eigen::Vector4f result4f = Eigen::Vector4f::Zero();
	result4f.segment(0,3) = jointTranslationDirection;

	result4f = getGlobalPose()*result4f;

	if (coordSystem)
	{
		result4f = coordSystem->getGlobalPose().inverse() * result4f;
	}

	return result4f.segment(0,3);
}



} // namespace
