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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010,2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_Transformation_h_
#define _VirtualRobot_Transformation_h_

#include "../VirtualRobotImportExport.h"
#include <Eigen/Core>

namespace VirtualRobot
{

class VIRTUAL_ROBOT_IMPORT_EXPORT Transformation
{
public:
	/*!
	Constructor
	*/
	Transformation();

	/*!
	*/
	virtual ~Transformation();

	Eigen::Matrix4f getPreJointTransformation();
	Eigen::Matrix4f getPostJointTransformation();
	Eigen::Vector3f getJointRotationAxis();
	Eigen::Vector3f getJointTranslationDirection();
};

} // namespace VirtualRobot

#endif // _VirtualRobot_Transformation_h_
