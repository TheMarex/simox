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
* @author     Peter Kaiser, Nikolaus Vahrenkamp
* @copyright  2011 Peter Kaiser, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_CogIK_h_
#define _VirtualRobot_CogIK_h_

#include "../VirtualRobotImportExport.h"

#include "../Nodes/RobotNode.h"
#include "../RobotNodeSet.h"
#include "JacobiProvider.h"
#include "DifferentialIK.h"

#include <boost/shared_ptr.hpp>

namespace VirtualRobot
{
class VIRTUAL_ROBOT_IMPORT_EXPORT CoMIK :
		public JacobiProvider,
		public boost::enable_shared_from_this<CoMIK>
{
public:
	/*!
		Initialize with a rns that contains joints and one that contains the bodies.
	*/
	CoMIK(RobotNodeSetPtr rnsJoints, RobotNodeSetPtr rnsBodies, RobotNodePtr coordSystem = RobotNodePtr());

	void setGoal(const Eigen::Vector2f &goal, float tolerance=5.0f);

	Eigen::MatrixXf getJacobianOfCoM(RobotNodePtr node);
	virtual Eigen::MatrixXf getJacobianMatrix();
	virtual Eigen::MatrixXf getJacobianMatrix(RobotNodePtr tcp); // ignored for CoM IK but needed for interface

	Eigen::VectorXf computeStep(float stepSize );
	bool computeSteps(float stepSize, float minumChange, int maxNStep);

	bool isValid(const Eigen::VectorXf &v) const;

	bool checkTolerances() const;
	void checkImprovements( bool enable );
	bool solveIK(float stepSize = 0.2f, float minChange = 0.0f, int maxSteps = 50);

private:
	RobotNodePtr coordSystem;
	RobotNodeSetPtr rnsBodies;

	std::vector< RobotNodePtr > bodyNodes;
	std::map< VirtualRobot::RobotNodePtr, std::vector<VirtualRobot::RobotNodePtr> > bodyNodeParents;

	float tolerance;
	bool checkImprovement;
	Eigen::Vector2f target;
};


typedef boost::shared_ptr<CoMIK> CoMIKPtr;

}


#endif
