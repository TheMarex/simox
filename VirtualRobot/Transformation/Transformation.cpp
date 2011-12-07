/**
* @package    VirtualRobot
* @author     Manfred Kroehnert <mkroehnert _at_ users dot sourceforge dot net>
* @copyright  2010 Manfred Kroehnert
*/


#include "Transformation.h"

namespace VirtualRobot {

Transformation::Transformation()
{
}
	
Transformation::~Transformation()
{
}

Eigen::Matrix4f Transformation::getPreJointTransformation()
{
	return Eigen::Matrix4f::Identity();
}

Eigen::Matrix4f Transformation::getPostJointTransformation()
{
	return Eigen::Matrix4f::Identity();
}

Eigen::Vector3f Transformation::getJointRotationAxis()
{
	return Eigen::Vector3f::Identity();
}

Eigen::Vector3f Transformation::getJointTranslationDirection()
{
	return Eigen::Vector3f::Identity();
}

} // namespace VirtualRobot
