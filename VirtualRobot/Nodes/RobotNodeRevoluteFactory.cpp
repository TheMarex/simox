/**
* @package    VirtualRobot
* @author     Manfred Kroehnert 
* @copyright  2010 Manfred Kroehnert
*/

#include "RobotNodeRevoluteFactory.h"
#include "RobotNode.h"
#include "RobotNodeRevolute.h"


namespace VirtualRobot {

RobotNodeRevoluteFactory::RobotNodeRevoluteFactory()
{
}


RobotNodeRevoluteFactory::~RobotNodeRevoluteFactory()
{
}


/**
 * This method creates a VirtualRobot::RobotNodeRevolute.
 *
 * \return instance of VirtualRobot::RobotNodeRevolute.
 */
RobotNodePtr RobotNodeRevoluteFactory::createRobotNode(RobotPtr robot, const std::string& nodeName, const std::vector<std::string>& childrenNames, VisualizationNodePtr visualizationModel, CollisionModelPtr collisionModel, float limitLow, float limitHigh, float jointValueOffset, const Eigen::Matrix4f& preJointTransform, const Eigen::Vector3f& axis, const Eigen::Matrix4f& postJointTransform, const Eigen::Vector3f& translationDirection, const SceneObject::Physics &p) const
{
	RobotNodePtr robotNode(new RobotNodeRevolute(robot, nodeName, childrenNames, limitLow, limitHigh, preJointTransform, axis, postJointTransform, visualizationModel, collisionModel, jointValueOffset, p));

	return robotNode;
}


/**
 * This method creates a VirtualRobot::RobotNodeRevolute from DH parameters.
 *
 * \return instance of VirtualRobot::RobotNodeRevolute.
 */
RobotNodePtr RobotNodeRevoluteFactory::createRobotNodeDH(RobotPtr robot, const std::string& nodeName, const std::vector<std::string>& childrenNames, VisualizationNodePtr visualizationModel, CollisionModelPtr collisionModel, float limitLow, float limitHigh, float jointValueOffset, const DHParameter& dhParameters, const SceneObject::Physics &p) const
{
	RobotNodePtr robotNode(new RobotNodeRevolute(robot, nodeName, childrenNames, limitLow, limitHigh, dhParameters.aMM(), dhParameters.dMM(), dhParameters.alphaRadian(), dhParameters.thetaRadian(), visualizationModel, collisionModel, jointValueOffset, p));

	return robotNode;

/*
	Eigen::Matrix4f preJointTransformation = Eigen::Matrix4f::Identity();	// no pre transformation
	Eigen::Vector3f jointRotationAxis = Eigen::Vector3f(0,0,1);			// rotation around z axis
	Eigen::Matrix4f postJointTransformation = dhParameters.thetaRotationRadian()* dhParameters.dTranslation() * dhParameters.aTranslation() * dhParameters.alphaRotationRadian();

	// unused but necessary for calling the method
	Eigen::Vector3f jointTranslationDirection = Eigen::Vector3f::Zero();

	return createRobotNode(robot, nodeName, childrenNames, visualizationModel, collisionModel, limitLow, limitHigh, jointValueOffset,
		preJointTransformation, jointRotationAxis, postJointTransformation, jointTranslationDirection);

	return createRobotNode(robot, nodeName, childrenNames, visualizationModel, collisionModel, limitLow, limitHigh, jointValueOffset, preJointTransformation, jointRotationAxis, postJointTransformation, jointTranslationDirection);
	*/
}


/**
 * register this class in the super class factory
 */
RobotNodeFactory::SubClassRegistry RobotNodeRevoluteFactory::registry(RobotNodeRevoluteFactory::getName(), &RobotNodeRevoluteFactory::createInstance);


/**
 * \return "revolute"
 */
std::string RobotNodeRevoluteFactory::getName() {return "revolute";}


/**
 * \return new instance of RobotNodeRevoluteFactory.
 */
boost::shared_ptr<RobotNodeFactory> RobotNodeRevoluteFactory::createInstance(void*)
{
    boost::shared_ptr<RobotNodeRevoluteFactory> revoluteNodeFactory(new RobotNodeRevoluteFactory());
    return revoluteNodeFactory;
}

} // namespace VirtualRobot
