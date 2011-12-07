/**
* @package    VirtualRobot
* @author     Manfred Kroehnert <mkroehnert _at_ users dot sourceforge dot net>
* @copyright  2010 Manfred Kroehnert
*/

#include "RobotNodePrismaticFactory.h"
#include "RobotNode.h"
#include "RobotNodePrismatic.h"


namespace VirtualRobot {

RobotNodePrismaticFactory::RobotNodePrismaticFactory()
{
}


RobotNodePrismaticFactory::~RobotNodePrismaticFactory()
{
}


/**
 * This method creates a VirtualRobot::RobotNodePrismatic.
 *
 * \return instance of VirtualRobot::RobotNodePrismatic.
 */
RobotNodePtr RobotNodePrismaticFactory::createRobotNode(RobotPtr robot, const std::string& nodeName, const std::vector<std::string>& childrenNames, VisualizationNodePtr visualizationModel, CollisionModelPtr collisionModel, float limitLow, float limitHigh, float jointValueOffset, const Eigen::Matrix4f& preJointTransform, const Eigen::Vector3f& axis, const Eigen::Matrix4f& postJointTransform, const Eigen::Vector3f& translationDirection, const SceneObject::Physics &p) const
{
	RobotNodePtr robotNode(new RobotNodePrismatic(robot, nodeName, childrenNames, limitLow, limitHigh, preJointTransform, translationDirection, postJointTransform, visualizationModel, collisionModel, jointValueOffset,p));

	return robotNode;
}


/**
 * This method creates a VirtualRobot::RobotNodePrismatic from DH parameters.
 *
 * \return instance of VirtualRobot::RobotNodePrismatic.
 */
RobotNodePtr RobotNodePrismaticFactory::createRobotNodeDH(RobotPtr robot, const std::string& nodeName, const std::vector<std::string>& childrenNames, VisualizationNodePtr visualizationModel, CollisionModelPtr collisionModel, float limitLow, float limitHigh, float jointValueOffset, const DHParameter& dhParameters, const SceneObject::Physics &p) const
{
	RobotNodePtr robotNode(new RobotNodePrismatic(robot, nodeName, childrenNames, limitLow, limitHigh, dhParameters.aMM(), dhParameters.dMM(), dhParameters.alphaRadian(), dhParameters.thetaRadian(), visualizationModel, collisionModel, jointValueOffset,p));

	return robotNode;

	/*
	// fixed rotation around theta
	Eigen::Matrix4f preJointTransformation = dhParameters.thetaRotationRadian();
	// joint setup
	Eigen::Vector3f jointTranslationDirection = Eigen::Vector3f(0,0,1);	// translation along the z axis
	// compute postJointTransformation
	Eigen::Matrix4f postJointTransformation = dhParameters.aTranslation() * dhParameters.alphaRotationRadian();

	// unused but necessary for calling the method
	Eigen::Vector3f jointRotationAxis = Eigen::Vector3f::Zero();

	return createRobotNode(robot, nodeName, childrenNames, visualizationModel, collisionModel, limitLow, limitHigh, jointValueOffset, preJointTransformation, jointRotationAxis, postJointTransformation, jointTranslationDirection);
	*/
}


/**
 * register this class in the super class factory
 */
RobotNodeFactory::SubClassRegistry RobotNodePrismaticFactory::registry(RobotNodePrismaticFactory::getName(), &RobotNodePrismaticFactory::createInstance);


/**
 * \return "prismatic"
 */
std::string RobotNodePrismaticFactory::getName() {return "prismatic";}


/**
 * \return new instance of RobotNodePrismaticFactory.
 */
boost::shared_ptr<RobotNodeFactory> RobotNodePrismaticFactory::createInstance(void*)
{
    boost::shared_ptr<RobotNodePrismaticFactory> prismaticNodeFactory(new RobotNodePrismaticFactory());
    return prismaticNodeFactory;
}

} // namespace VirtualRobot
