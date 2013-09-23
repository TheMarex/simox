#include "DynamicsObject.h"

#include <VirtualRobot/SceneObject.h>

namespace SimDynamics {

DynamicsObject::DynamicsObject(VirtualRobot::SceneObjectPtr o)
{
	THROW_VR_EXCEPTION_IF(!o,"NULL object");
	sceneObject = o;
}
	
DynamicsObject::~DynamicsObject()
{
}

std::string DynamicsObject::getName() const
{
	return sceneObject->getName();
}

VirtualRobot::SceneObject::Physics::SimulationType DynamicsObject::getSimType() const
{
    return sceneObject->getSimulationType();
}

void DynamicsObject::setPose( const Eigen::Matrix4f &pose )
{
    if (sceneObject->getSimulationType()==VirtualRobot::SceneObject::Physics::eStatic)
	{
		VR_ERROR << "Could not move static object, aborting..." << endl;
		return;
	}
	//sceneObject->setGlobalPose(pose);
}

void DynamicsObject::setPosition( const Eigen::Vector3f &posMM )
{
	Eigen::Matrix4f pose = sceneObject->getGlobalPoseVisualization();
	pose.block(0,3,3,1) = posMM;
	setPose(pose);
}

VirtualRobot::SceneObjectPtr DynamicsObject::getSceneObject()
{
	return sceneObject;
}

Eigen::Vector3f DynamicsObject::getLinearVelocity()
{
	return Eigen::Vector3f::Zero();
}

Eigen::Vector3f DynamicsObject::getAngularVelocity()
{
    return Eigen::Vector3f::Zero();
}

void DynamicsObject::applyForce(const Eigen::Vector3f &force)
{

}

void DynamicsObject::applyTorque(const Eigen::Vector3f &torque)
{

}


} // namespace SimDynamics
