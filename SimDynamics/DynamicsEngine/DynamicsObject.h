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
* @package    SimDynamics
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _SimDynamics_DynamicsObject_h_
#define _SimDynamics_DynamicsObject_h_

#include "../SimDynamics.h"

namespace SimDynamics
{
class SIMDYNAMICS_IMPORT_EXPORT DynamicsObject
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum SimulationType 
	{
		eStatic,		// cannot move, but collide
		eKinematic,		// can be moved, but no dynamics
		eDynamic		// full dynamic simulation
	};
	/*!
		Constructor
	*/
	DynamicsObject(VirtualRobot::SceneObjectPtr o, SimulationType type = eDynamic);

	/*!
	*/
	virtual ~DynamicsObject();

	std::string getName() const;

	SimulationType getSimType() const;

	/*!
		Set world position [MM].
	*/
	virtual void setPosition(const Eigen::Vector3f &posMM);

	/*!
		Set world pose [mm].
	*/
	virtual void setPose(const Eigen::Matrix4f &pose);

	VirtualRobot::SceneObjectPtr getSceneObject();

protected:

	VirtualRobot::SceneObjectPtr sceneObject;
	SimulationType simulationType;
};

typedef boost::shared_ptr<DynamicsObject> DynamicsObjectPtr;

} // namespace SimDynamics

#endif // _SimDynamics_DynamicsObject_h_
