#include "CoMIK.h"
#include "DifferentialIK.h"
#include "../Nodes/RobotNodeRevolute.h"
#include "../Nodes/RobotNodePrismatic.h"
#include "../VirtualRobotException.h"
#include "../Robot.h"

#include <float.h>

namespace VirtualRobot
{
	CoMIK::CoMIK(RobotNodeSetPtr rnsJoints, RobotNodeSetPtr rnsBodies, RobotNodePtr coordSystem)
	: JacobiProvider(rnsJoints), coordSystem(coordSystem)
{
	VR_ASSERT(rns);
	VR_ASSERT(rnsBodies);
	checkImprovement = false;
	this->rnsBodies = rnsBodies;

	bodyNodes = rnsBodies->getAllRobotNodes();
	for (size_t i = 0; i < bodyNodes.size(); i++)
	{
		// get all joints that influence the body
		std::vector<RobotNodePtr> parentsN = bodyNodes[i]->getAllParents(rns);
		bodyNodeParents[bodyNodes[i]] = parentsN;
	}
	if (rnsBodies->getMass()==0)
	{
		VR_ERROR << "The RNS does not contain any bodies or masses are not specified (mass==0)" << endl;
	}
}

void CoMIK::setGoal(const Eigen::Vector2f &goal, float tolerance)
{
	target = goal;
	this->tolerance = tolerance;
	initialized = true;
}

Eigen::MatrixXf CoMIK::getJacobianOfCoM(RobotNodePtr node)
{
	// Get number of degrees of freedom
	size_t nDoF = rns->getAllRobotNodes().size();

	// Create matrices for the position and the orientation part of the jacobian.
	Eigen::MatrixXf position = Eigen::MatrixXf::Zero(3,nDoF);

	const std::vector<RobotNodePtr> parentsN = bodyNodeParents[node];

	// Iterate over all degrees of freedom
	for (size_t i = 0; i < nDoF; i++)
	{
		RobotNodePtr dof = rns->getNode(i);// bodyNodes[i];

		// Check if the tcp is affected by this DOF
		if (find(parentsN.begin(),parentsN.end(),dof) != parentsN.end() )
		{
			// Calculus for rotational joints is different as for prismatic joints.
			if (dof->isRotationalJoint())
			{
				// get axis
				boost::shared_ptr<RobotNodeRevolute> revolute
					= boost::dynamic_pointer_cast<RobotNodeRevolute>(dof);
				THROW_VR_EXCEPTION_IF(!revolute,"Internal error: expecting revolute joint");
				// todo: find a better way of handling different joint types
				Eigen::Vector3f axis = revolute->getJointRotationAxis(coordSystem);

				// For CoM-Jacobians only the positional part is necessary
				Eigen::Vector3f toTCP = node->getCoMLocal() + node->getGlobalPose().block(0,3,3,1)
					- dof->getGlobalPose().block(0,3,3,1);
				position.block(0,i,3,1) = axis.cross(toTCP);
			}
			else if (dof->isTranslationalJoint())
			{
				// -> prismatic joint
				boost::shared_ptr<RobotNodePrismatic> prismatic
						= boost::dynamic_pointer_cast<RobotNodePrismatic>(dof);
				THROW_VR_EXCEPTION_IF(!prismatic,"Internal error: expecting prismatic joint");
				// todo: find a better way of handling different joint types
				Eigen::Vector3f axis = prismatic->getJointTranslationDirection(coordSystem);

				position.block(0,i,3,1) = axis;
			}
		}
	}

	Eigen::MatrixXf result(2,nDoF);
	result.row(0) = position.row(0);
	result.row(1) = position.row(1);

	//cout << "Position, jac:\n" << position << endl;

	return result;
}

Eigen::MatrixXf CoMIK::getJacobianMatrix(RobotNodePtr tcp)
{
	// ignoring tcp
	return getJacobianMatrix();
}

Eigen::MatrixXf CoMIK::getJacobianMatrix()
{
	//const std::vector<RobotNodePtr> nodes = rns->getAllRobotNodes();

	Eigen::MatrixXf Jsum(0,0);
	for(std::vector<RobotNodePtr>::const_iterator n = bodyNodes.begin(); n != bodyNodes.end(); n++)
	{
		// Retrieve (positional) Jacobian for this node's CoM
		// This Jacobian is already projected to the X-Y-plane
		Eigen::MatrixXf J = getJacobianOfCoM(*n);

		// Sum up the Jacobians weighted by the respective node's mass
		if(Jsum.rows() == 0)
			Jsum = (*n)->getMass() * J;
		else
			Jsum += (*n)->getMass() * J;
	}

	if (rnsBodies->getMass()!=0)
		Jsum /= rnsBodies->getMass();
	return Jsum;
}

Eigen::VectorXf CoMIK::getError(float stepsize)
{
	return (target - rnsBodies->getCoM().head(2));
}

Eigen::VectorXf CoMIK::computeStep(float stepSize )
{
	Eigen::Vector2f error = getError(stepSize);
	return getPseudoInverseJacobianMatrix() * error;

}

bool CoMIK::checkTolerances() const
{
	float error = (target - rnsBodies->getCoM().head(2)).norm();
	return error < tolerance;
}

void CoMIK::checkImprovements( bool enable )
{
	checkImprovement = enable;
}

bool CoMIK::isValid(const Eigen::VectorXf &v) const
{
	return MathTools::isValid(v);
}

bool CoMIK::computeSteps(float stepSize, float minumChange, int maxNStep) 
{
	std::vector<RobotNodePtr> rn = rns->getAllRobotNodes();
	RobotPtr robot = rns->getRobot();
	std::vector<float> jv(rns->getSize(),0.0f);
	int step = 0;
	checkTolerances();
	float lastDist = FLT_MAX;

	while (step<maxNStep)
	{
		Eigen::VectorXf dTheta = this->computeStep(stepSize);

		// Check for singularities
		if(!isValid(dTheta))
		{
			VR_INFO << "Singular Jacobian" << endl;
			return false;
		}

		for (unsigned int i=0; i<rn.size();i++)
			jv[i] = (rn[i]->getJointValue() + dTheta[i]);
			//rn[i]->setJointValue(rn[i]->getJointValue() + dTheta[i]);
		robot->setJointValues(rns,jv);
		// check tolerances
		if (checkTolerances())
		{
			VR_INFO << "Tolerances ok, loop:" << step << endl;
			return true;
		}
		float d = dTheta.norm();
		if (dTheta.norm()<minumChange)
		{
			VR_INFO << "Could not improve result any more (dTheta.norm()=" << d << "), loop:" << step << endl;
			return false;
		}
		if (checkImprovement && d>lastDist)
		{
			VR_INFO << "Could not improve result any more (dTheta.norm()=" << d << ", last loop's norm:" << lastDist << "), loop:" << step << endl;
			return false;
		}
		lastDist = d;
		step++;
	}

	VR_INFO << "IK failed, loop:" << step << endl;
	VR_INFO << "error:" << (target - rns->getCoM().head(2)).norm() << endl;
	return false;
}

bool CoMIK::solveIK( float stepSize, float minChange, int maxSteps)
{
	return computeSteps(stepSize,minChange,maxSteps);
}


}
