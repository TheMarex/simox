
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
	) : RobotNode(rob,name,jointLimitLo,jointLimitHi,visualization,collisionModel,jointValueOffset,p,colChecker)

{
	initialized = false;
	optionalDHParameter.isSet = false;
	this->preJointTransformation = preJointTransform;
	this->postJointTransformation = postJointTransform;
	//this->setPreJointTransformation(preJointTransform);
	this->jointTranslationDirection = translationDirection;
	//this->setPostJointTransformation(postJointTransform);
}

RobotNodePrismatic::RobotNodePrismatic(RobotWeakPtr rob, 
	const std::string &name,
	float jointLimitLo,
	float jointLimitHi,
	float a, float d, float alpha, float theta,
	VisualizationNodePtr visualization, 
	CollisionModelPtr collisionModel,
	float jointValueOffset,
	const SceneObject::Physics &p,
	CollisionCheckerPtr colChecker
	) : RobotNode(rob,name,jointLimitLo,jointLimitHi,visualization,collisionModel,jointValueOffset,p,colChecker)

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
	this->preJointTransformation = RotTheta;
	// joint setup
	jointTranslationDirection = Eigen::Vector3f(0,0,1);	// translation along the z axis
	// compute postJointTransformation
	this->postJointTransformation = TransD*TransA*RotAlpha;
}

RobotNodePrismatic::~RobotNodePrismatic()
{
}


bool RobotNodePrismatic::initialize(SceneObjectPtr parent, const std::vector<SceneObjectPtr> &children)
{
	return RobotNode::initialize(parent,children);
}

void RobotNodePrismatic::updateTransformationMatrices(const Eigen::Matrix4f &parentPose)
{
	Eigen::Affine3f tmpT(Eigen::Translation3f((this->getJointValue()+jointValueOffset)*jointTranslationDirection));
	globalPose = parentPose * getPreJointTransformation() * tmpT.matrix();
	globalPosePostJoint = globalPose*getPostJointTransformation();
}

void RobotNodePrismatic::print( bool printChildren, bool printDecoration ) const
{
	ReadLockPtr lock = getRobot()->getReadLock();
	
	if (printDecoration)
		cout << "******** RobotNodePrismatic ********" << endl;

	RobotNode::print(false,false);

	cout << "* jointTranslationDirection: " << jointTranslationDirection[0] << ", " << jointTranslationDirection[1] << "," << jointTranslationDirection[2] << endl;

	if (printDecoration)
		cout << "******** End RobotNodePrismatic ********" << endl;
	

	std::vector< SceneObjectPtr > children = this->getChildren();
	if (printChildren)
		std::for_each(children.begin(), children.end(), boost::bind(&SceneObject::print, _1, true, true));
}

RobotNodePtr RobotNodePrismatic::_clone(const RobotPtr newRobot, /*const std::vector<std::string> newChildren,*/ const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel, CollisionCheckerPtr colChecker)
{
	RobotNodePtr result;
	ReadLockPtr lock = getRobot()->getReadLock();

	if (optionalDHParameter.isSet)
		result.reset(new RobotNodePrismatic(newRobot,name, jointLimitLo,jointLimitHi,optionalDHParameter.aMM(),optionalDHParameter.dMM(), optionalDHParameter.alphaRadian(), optionalDHParameter.thetaRadian(),visualizationModel,collisionModel, jointValueOffset,physics,colChecker));
	else
		result.reset(new RobotNodePrismatic(newRobot,name,jointLimitLo,jointLimitHi,getPreJointTransformation(),jointTranslationDirection,getPostJointTransformation(),visualizationModel,collisionModel,jointValueOffset,physics,colChecker));
	return result;
}

bool RobotNodePrismatic::isTranslationalJoint() const
{
	return true;
}


Eigen::Vector3f RobotNodePrismatic::getJointTranslationDirection(const SceneObjectPtr coordSystem) const
{
	ReadLockPtr lock = getRobot()->getReadLock();
	Eigen::Vector4f result4f = Eigen::Vector4f::Zero();
	result4f.segment(0,3) = jointTranslationDirection;

	result4f = getGlobalPose()*result4f;

	if (coordSystem)
	{
		result4f = coordSystem->getGlobalPose().inverse() * result4f;
	}

	return result4f.segment(0,3);
}

void RobotNodePrismatic::updateVisualizationPose( const Eigen::Matrix4f &globalPose, bool updateChildren /*= false*/ )
{
	RobotNode::updateVisualizationPose(globalPose,updateChildren);

	// compute the jointValue from pose
    Eigen::Matrix4f initFrame;
    if (this->getParent())
        initFrame = this->getParent()->getGlobalPose() * getPreJointTransformation();
    else
        initFrame = getPreJointTransformation();

    Eigen::Matrix4f localPose = initFrame.inverse() * globalPose;

	Eigen::Vector3f v = localPose.block(0,3,3,1);

	// project on directionVector
	MathTools::Line l(Eigen::Vector3f::Zero(),jointTranslationDirection);
	Eigen::Vector3f v_on_line = MathTools::nearestPointOnLine(l,v);
	float dist = v_on_line.norm();

	// check pos/neg direction
	if (v_on_line.normalized().dot(jointTranslationDirection.normalized()) < 0)
	{
		// pointing in opposite direction
		dist = -dist;
	}

	// consider offset
	jointValue = dist - jointValueOffset;
}

Eigen::Vector3f RobotNodePrismatic::getJointTranslationDirectionJointCoordSystem() const
{
	return jointTranslationDirection;
}



} // namespace
