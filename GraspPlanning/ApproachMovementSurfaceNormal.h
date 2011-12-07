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
* @package    GraspStudio
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef __APPROACH_MOVEMENT_GENERATOR_SURFACE_NORMAL_H__
#define __APPROACH_MOVEMENT_GENERATOR_SURFACE_NORMAL_H__

#include "GraspStudio.h"
#include "ApproachMovementGenerator.h"
#include <VirtualRobot/SceneObject.h>
#include <vector>

namespace GraspStudio
{
	/*!
	*
	*
	* This class generates grasping configs by sampling a random surface position of the object and setting the EEF to a surface normal aligned position.
	* The remaining free DoF (the rotation around the normal) is set randomly. Then the EEF is moved along the normal until 
	* a collision is detected or the GCP hits the object.
	* If needed, the EEF is moved back until a collision-free pose is found.
	*
	* Internally the EEF is cloned.
	*
	*/
class GRASPSTUDIO_IMPORT_EXPORT ApproachMovementSurfaceNormal : public ApproachMovementGenerator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
	    To generate approach movements an object and an end effector has to be specified.
	*/
	ApproachMovementSurfaceNormal(VirtualRobot::SceneObjectPtr object, VirtualRobot::EndEffectorPtr eef);
	//! destructor
	virtual ~ApproachMovementSurfaceNormal();

	//! Creates a new pose for approaching
	virtual Eigen::Matrix4f createNewApproachPose();

	//!  Returns a position with normal on the surface of the object
	bool getPositionOnObject(Eigen::Vector3f &storePos, Eigen::Vector3f& storeApproachDir);

	//! Sets EEF to a position so that the Z component of the GCP coord system is aligned with -approachDir
	bool setEEFToApproachPose(const Eigen::Vector3f &position, const Eigen::Vector3f &approachDir);

	void moveEEFAway(const Eigen::Vector3f &approachDir, float step, int maxLoops = 1000);

protected:

	void openHand();
};
}

#endif /* __APPROACH_MOVEMENT_GENERATOR_SURFACE_NORMAL_H__ */
