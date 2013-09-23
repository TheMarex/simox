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
#include <VirtualRobot/SceneObject.h>

namespace SimDynamics
{
class SIMDYNAMICS_IMPORT_EXPORT DynamicsObject
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*enum SimulationType
	{
		eStatic,		// cannot move, but collide
		eKinematic,		// can be moved, but no dynamics
		eDynamic		// full dynamic simulation
    };*/
	/*!
		Constructor
	*/
    DynamicsObject(VirtualRobot::SceneObjectPtr o);//, SimulationType type = eDynamic);

	/*!
	*/
	virtual ~DynamicsObject();

	std::string getName() const;

    VirtualRobot::SceneObject::Physics::SimulationType getSimType() const;

	/*!
		Set world position [MM].
	*/
	virtual void setPosition(const Eigen::Vector3f &posMM);

	/*!
		Set world pose [mm].
	*/
	virtual void setPose(const Eigen::Matrix4f &pose);

	VirtualRobot::SceneObjectPtr getSceneObject();

	virtual Eigen::Vector3f getLinearVelocity();
	virtual Eigen::Vector3f getAngularVelocity();

    /*!
     * \brief applyForce Applies an external force on this object. The force is applied at the CoM position.
     * \param force The force to apply (value with respect to one second). The force will be deleted after one simulation step.
     */
    virtual void applyForce(const Eigen::Vector3f &force);

    /*!
     * \brief applyTorque Applies an external torque on this object. The torque is applied at the CoM position.
     * \param torque The torque to apply (value with respect to one second). The torque will be deleted after one simulation step.
     */
    virtual void applyTorque(const Eigen::Vector3f &torque);


protected:

	VirtualRobot::SceneObjectPtr sceneObject;
    //SimulationType simulationType;
};

typedef boost::shared_ptr<DynamicsObject> DynamicsObjectPtr;

} // namespace SimDynamics

#endif // _SimDynamics_DynamicsObject_h_
