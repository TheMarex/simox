#include "CoMIK.h"
#include "DifferentialIK.h"
#include "../Nodes/RobotNodeRevolute.h"
#include "../Nodes/RobotNodePrismatic.h"
#include "../VirtualRobotException.h"

#include <float.h>

namespace VirtualRobot
{
CoMIK::CoMIK(RobotNodeSetPtr rns, RobotNodePtr coordSystem)
	: m_CoordSystem(coordSystem)
{
	m_RobotNodeSet = rns;
	m_CheckImprovement = false;
	if (rns)
	{
		nodes = m_RobotNodeSet->getAllRobotNodes();
		for (size_t i = 0; i < nodes.size(); i++)
		{
			std::vector<RobotNodePtr> parentsN = nodes[i]->getAllParents(m_RobotNodeSet);
			parents[nodes[i]] = parentsN;
		}
	}
			
}

void CoMIK::setGoal(const Eigen::Vector2f &goal, float tolerance)
{
	m_Target = goal;
	m_Tolerance = tolerance;
}

Eigen::MatrixXf CoMIK::getJacobianOfCoM(RobotNodePtr node)
{
	// Get number of degrees of freedom
	size_t nDoF = m_RobotNodeSet->getAllRobotNodes().size();

	// Create matrices for the position and the orientation part of the jacobian.
	Eigen::MatrixXf position = Eigen::MatrixXf::Zero(3,nDoF);


	// Iterate over all degrees of freedom
	for (size_t i = 0; i < nDoF; i++)
	{
		RobotNodePtr dof = nodes[i];
		const std::vector<RobotNodePtr> parentsN = parents[node];

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
				Eigen::Vector3f axis = revolute->getJointRotationAxis(m_CoordSystem);

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
				Eigen::Vector3f axis = prismatic->getJointTranslationDirection(m_CoordSystem);

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

Eigen::MatrixXf CoMIK::getJacobianMatrix()
{
	//const std::vector<RobotNodePtr> nodes = m_RobotNodeSet->getAllRobotNodes();

	Eigen::MatrixXf Jsum(0,0);
	for(std::vector<RobotNodePtr>::const_iterator n = nodes.begin(); n != nodes.end(); n++)
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

	if (m_RobotNodeSet->getMass()!=0)
		Jsum /= m_RobotNodeSet->getMass();
	return Jsum;
}

Eigen::MatrixXf CoMIK::getPseudoInverseJacobianMatrix()
{
	Eigen::MatrixXf Jacobian = this->getJacobianMatrix();
	bool isV1 = MathTools::isValid(Jacobian);
	Eigen::MatrixXf JacobianT = Jacobian.transpose();
	bool isV2 = MathTools::isValid(JacobianT);
	Eigen::MatrixXf JacJacobianT = Jacobian*JacobianT;
	bool isV3 = MathTools::isValid(JacJacobianT);
	Eigen::MatrixXf JacJacobianTI = JacJacobianT.inverse();
	bool isV4 = MathTools::isValid(JacJacobianTI);
	if (!isV4)
	{
		cout << "JacJacobianT: " << JacJacobianT << endl;
		cout << "JacJacobianTI: " << JacJacobianTI << endl;
		float d = JacJacobianT.determinant();
		cout << "d:" << d << endl;

	}
	Eigen::MatrixXf pseudo = Jacobian.transpose() * (Jacobian*Jacobian.transpose()).inverse();
	return pseudo;
}

Eigen::VectorXf CoMIK::computeStep(float stepSize )
{
	Eigen::Vector2f error = (m_Target - m_RobotNodeSet->getCoM().head(2)) * stepSize;
	return getPseudoInverseJacobianMatrix() * error;

}

bool CoMIK::checkTolerances() const
{
	float error = (m_Target - m_RobotNodeSet->getCoM().head(2)).norm();
	return error < m_Tolerance;
}

void CoMIK::checkImprovements( bool enable )
{
	m_CheckImprovement = enable;
}

bool CoMIK::isValid(const Eigen::VectorXf &v) const
{
	return MathTools::isValid(v);
}

bool CoMIK::computeSteps(float stepSize, float minumChange, int maxNStep) 
{
	std::vector<RobotNodePtr> rn = m_RobotNodeSet->getAllRobotNodes();
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
			rn[i]->setJointValue(rn[i]->getJointValue() + dTheta[i]);

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
		if (m_CheckImprovement && d>lastDist)
		{
			VR_INFO << "Could not improve result any more (dTheta.norm()=" << d << ", last loop's norm:" << lastDist << "), loop:" << step << endl;
			return false;
		}
		lastDist = d;
		step++;
	}

	VR_INFO << "IK failed, loop:" << step << endl;
	VR_INFO << "error:" << (m_Target - m_RobotNodeSet->getCoM().head(2)).norm() << endl;
	return false;
}

bool CoMIK::solveIK( float stepSize, float minChange, int maxSteps)
{
	return computeSteps(stepSize,minChange,maxSteps);
}


}
