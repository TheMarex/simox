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
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_BoundingBox_h_
#define _VirtualRobot_BoundingBox_h_

#include "VirtualRobotImportExport.h"
#include "MathTools.h"

#include <Eigen/Core>
#include <vector>

namespace VirtualRobot {

class VIRTUAL_ROBOT_IMPORT_EXPORT BoundingBox
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	BoundingBox();
	BoundingBox(const std::vector< Eigen::Vector3f > &p);

	bool planeGoesThrough(const VirtualRobot::MathTools::Plane &p);

	std::vector <Eigen::Vector3f> getPoints();

	void print();

	void addPoints( const std::vector < Eigen::Vector3f > &p );
	void addPoint (const Eigen::Vector3f &p);

	Eigen::Vector3f min;
	Eigen::Vector3f max;
};

} // namespace VirtualRobot

#endif /* _VirtualRobot_BoundingBox_h_ */
