
#include "RobotNodeRevolute.h"
#include "../Robot.h"
#include <cmath>
#include <algorithm>
#include <boost/bind.hpp>
 #include <Eigen/Geometry> 
#include "../VirtualRobotException.h"

#include "ConditionedLock.h"

namespace VirtualRobot {


RobotNodeRevolute::RobotNodeRevolute(RobotWeakPtr rob, 
									   const std::string &name,
									   const std::vector<std::string> &childrenNames,
									   float jointLimitLo,
									   float jointLimitHi,
									   const Eigen::Matrix4f &preJointTransform,			//!< This transformation is applied before the translation of the joint is done
									   const Eigen::Vector3f &axis,			//!< This is the direction of the translation
									   const Eigen::Matrix4f &postJointTransform,			//!< This is an additional transformation, that is applied after the translation is done
									   VisualizationNodePtr visualization, 
									   CollisionModelPtr collisionModel,
									   float jointValueOffset,
									   const SceneObject::Physics &p,
									   CollisionCheckerPtr colChecker
									   ) : RobotNode(rob,name,childrenNames,jointLimitLo,jointLimitHi,visualization,collisionModel,jointValueOffset,p,colChecker)

{
	initialized = false;
	optionalDHParameter.isSet = false;
	this->setPreJointTransformation(preJointTransform);
	jointRotationAxis = axis;
	this->setPostJointTransformation(postJointTransform);
	/*preJointTransformation = Eigen::Matrix4f::Identity();
	postJointTransformation = Eigen::Matrix4f::Identity();
	postJointTransformation.block<3,1>(0,3) = postJointTranslation;*/
//	postJointTransformation(1,3) = postJointTranslation(2);
//	postJointTransformation(2,3) = postJointTranslation(3);

}

RobotNodeRevolute::RobotNodeRevolute(RobotWeakPtr rob, 
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

	this->setPreJointTransformation(RotTheta);//Eigen::Matrix4f::Identity();	// no pre transformation->why?
	jointRotationAxis = Eigen::Vector3f(0,0,1);			// rotation around z axis
	this->setPostJointTransformation(TransD*TransA*RotAlpha);
}


RobotNodeRevolute::~RobotNodeRevolute()
{
}

void RobotNodeRevolute::reset()
{
	{
		WriteLock w(mutex,this->robot.lock()->isThreadsafe());
		jointRotationAxis = Eigen::Vector3f(1,0,0);
	}
	RobotNode::reset();
}

bool RobotNodeRevolute::initialize(RobotNodePtr parent, bool initializeChildren)
{
	return RobotNode::initialize(parent,initializeChildren);
}

void RobotNodeRevolute::updateTransformationMatrices()
{
	{
		
		WriteLock w(mutex,this->robot.lock()->isThreadsafe());
		
		if (this->getParent())
			globalPose = this->getParent()->getGlobalPose() * getPreJointTransformation();
		else
			globalPose = getPreJointTransformation();

		Eigen::Affine3f tmpT(Eigen::AngleAxisf(this->getJointValue()+jointValueOffset,jointRotationAxis));
		globalPose *= tmpT.matrix();

		globalPosePostJoint = globalPose*getPostJointTransformation();
	}
	// update collision and visualization model
	// here we do not consider the postJointTransformation, since it already defines the transformation to the next joint.
	SceneObject::setGlobalPose(globalPose);
}

void RobotNodeRevolute::updateTransformationMatrices(const Eigen::Matrix4f &globalPose)
{
	THROW_VR_EXCEPTION_IF(this->getParent(),"This method could only be called on RobotNodes without parents.");
	{
		
		WriteLock w(mutex,this->robot.lock()->isThreadsafe());
		this->globalPose = globalPose * getPreJointTransformation();

		Eigen::Affine3f tmpT(Eigen::AngleAxisf(this->getJointValue()+jointValueOffset,jointRotationAxis));
		this->globalPose *= tmpT.matrix();

		globalPosePostJoint = this->globalPose*getPostJointTransformation();
	}
	// update collision and visualization model
	// here we do not consider the postJointTransformation, since it already defines the transformation to the next joint.
	SceneObject::setGlobalPose(this->globalPose);
}


void RobotNodeRevolute::print( bool printChildren, bool printDecoration ) const
{
	ReadLock lock(mutex,this->robot.lock()->isThreadsafe());

	if (printDecoration)
		cout << "******** RobotNodeRevolute ********" << endl;

	RobotNode::print(false,false);

	cout << "* JointRotationAxis: " << jointRotationAxis[0] << ", " << jointRotationAxis[1] << ", " << jointRotationAxis[2] << endl;

	if (printDecoration)
		cout << "******** End RobotNodeRevolute ********" << endl;

	std::vector< RobotNodePtr > children = this->getChildren();
	if (printChildren)
		std::for_each(children.begin(), children.end(), boost::bind(&RobotNode::print, _1, true, true));
}

RobotNodePtr RobotNodeRevolute::_clone(const RobotPtr newRobot, const std::vector<std::string> newChildren, const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel)
{
	ReadLock lock(mutex,this->robot.lock()->isThreadsafe());
	RobotNodePtr result;

	if (optionalDHParameter.isSet)
		result.reset(new RobotNodeRevolute(newRobot,name,newChildren, jointLimitLo,jointLimitHi,optionalDHParameter.aMM(),optionalDHParameter.dMM(), optionalDHParameter.alphaRadian(), optionalDHParameter.thetaRadian(),visualizationModel,collisionModel, jointValueOffset,physics));
	else
		result.reset(new RobotNodeRevolute(newRobot,name,newChildren,jointLimitLo,jointLimitHi,getPreJointTransformation(),jointRotationAxis,getPostJointTransformation(),visualizationModel,collisionModel,jointValueOffset,physics));
	return result;
}

bool RobotNodeRevolute::isRotationalJoint() const
{
	return true;
}

Eigen::Vector3f RobotNodeRevolute::getJointRotationAxis(const SceneObjectPtr coordSystem) const
{
	ReadLock lock(mutex,this->robot.lock()->isThreadsafe());
	//Eigen::Vector3f res = toGlobalCoordinateSystem(jointRotationAxis);
	Eigen::Vector4f result4f = Eigen::Vector4f::Zero();
	result4f.segment(0,3) = jointRotationAxis;
	result4f = getGlobalPoseJoint()*result4f;

	if (coordSystem)
	{
		//res = coordSystem->toLocalCoordinateSystem(res);
		result4f = coordSystem->getGlobalPose().inverse() * result4f;
	}

	return result4f.block(0,0,3,1);
}

} // namespace
