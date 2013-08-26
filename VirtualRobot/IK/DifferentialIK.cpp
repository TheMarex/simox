#include <Eigen/Geometry>
#include "DifferentialIK.h"
#include "../Robot.h"
#include "../VirtualRobotException.h"
#include "../Nodes/RobotNodePrismatic.h"
#include "../Nodes/RobotNodeRevolute.h"
#include "../VirtualRobotException.h"
#include "../CollisionDetection/CollisionChecker.h"
#include <boost/format.hpp>

#include <boost/bind.hpp>

#include <boost/math/special_functions/fpclassify.hpp>

#include <algorithm>
#include <float.h>

//#define CHECK_PERFORMANCE

using namespace Eigen;
namespace VirtualRobot
{

DifferentialIK::DifferentialIK(RobotNodeSetPtr _rns, RobotNodePtr _coordSystem, JacobiProvider::InverseJacobiMethod invJacMethod) :
    JacobiProvider(_rns,invJacMethod), coordSystem(_coordSystem), nRows(0)
{
	if (!rns)
		THROW_VR_EXCEPTION("Null data");
	checkImprovement = false;
	nodes =  rns->getAllRobotNodes();
	for (size_t i=0; i<nodes.size();i++)
	{
		std::vector<RobotNodePtr> p = nodes[i]->getAllParents(rns);
		p.push_back(nodes[i]);// if the tcp is not fixed, it must be considered for calculating the Jacobian
		parents[nodes[i]] = p;
	}
	convertMMtoM = false;
	verbose = false;
}


void DifferentialIK::setGoal(const Eigen::Matrix4f &goal, RobotNodePtr tcp, IKSolver::CartesianSelection mode, float tolerancePosition, float toleranceRotation)
{
	if (!tcp) tcp = this->getDefaultTCP();

	// tcp not in list yet?
	if ( find(tcp_set.begin(),tcp_set.end(),tcp)  == tcp_set.end()  )
		tcp_set.push_back(tcp);
	
	this->targets[tcp] = goal;
	this->modes[tcp] = mode;	
	this->tolerancePosition[tcp] = tolerancePosition;	
	this->toleranceRotation[tcp] = toleranceRotation;	

	std::vector<RobotNodePtr> p = tcp->getAllParents(rns);
	p.push_back(tcp);
	parents[tcp] = p;
	
	this->setNRows();
}	

MatrixXf DifferentialIK::getJacobianMatrix()
{
	return getJacobianMatrix(RobotNodePtr());
}

MatrixXf DifferentialIK::getJacobianMatrix(RobotNodePtr tcp)
{
	return getJacobianMatrix(tcp,IKSolver::All);
}

MatrixXf DifferentialIK::getJacobianMatrix(IKSolver::CartesianSelection mode)
{
	return getJacobianMatrix(RobotNodePtr(),mode);
}

MatrixXf DifferentialIK::getJacobianMatrix(RobotNodePtr tcp, IKSolver::CartesianSelection mode)
{
	// Get number of degrees of freedom
	size_t nDoF = nodes.size();
	
	// Create matrices for the position and the orientation part of the jacobian.
	MatrixXf position = MatrixXf::Zero(3,nDoF);
	MatrixXf orientation = MatrixXf::Zero(3,nDoF);
	
	if (!tcp) tcp = this->getDefaultTCP();
//	THROW_VR_EXCEPTION_IF(!tcp,boost::format("No tcp defined in node set \"%1%\" of robot %2% (DifferentialIK::%3% )") % this->rns->getName() % this->rns->getRobot()->getName() % BOOST_CURRENT_FUNCTION);
		


	// check if we already computed the parents for tcp
	if (parents.find(tcp) == parents.end())
	{
		parents[tcp] = tcp->getAllParents(rns);
		parents[tcp].push_back(tcp);
	}

	// Iterate over all degrees of freedom
	for (size_t i=0; i<nDoF;i++){

#ifdef CHECK_PERFORMANCE
        clock_t startT = clock();
#endif

		RobotNodePtr dof = this->nodes[i];
		//std::vector<RobotNodePtr> parents = parents[tcp];//tcp->getAllParents(this->rns);

		//check if the tcp is affected by this DOF
		if ( find(parents[tcp].begin(),parents[tcp].end(),dof)  != parents[tcp].end() )
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
				
				// if necessary calculate the position part of the Jacobian
				if (mode & IKSolver::Position) 
				{
					Vector3f toTCP;
					if (coordSystem)
					{
						toTCP = coordSystem->toLocalCoordinateSystem(tcp->getGlobalPose()).block(0,3,3,1) 
						- coordSystem->toLocalCoordinateSystem(dof->getGlobalPoseJoint()).block(0,3,3,1);
					} else
					{
						toTCP = tcp->getGlobalPose().block(0,3,3,1) 
							- dof->getGlobalPoseJoint().block(0,3,3,1);
					}
					if (convertMMtoM)
						toTCP /= 1000.0f;
					//cout << "toTCP: " << tcp->getName() << endl;
					//cout << toTCP << endl;
					position.block(0,i,3,1) = axis.cross(toTCP);
				}
				// and the orientation part
				if (mode & IKSolver::Orientation)
					orientation.block(0,i,3,1) = axis;
			}
			else if (dof->isTranslationalJoint()) 
			{ 
				// -> prismatic joint
                boost::shared_ptr<RobotNodePrismatic> prismatic
                        = boost::dynamic_pointer_cast<RobotNodePrismatic>(dof);
				THROW_VR_EXCEPTION_IF(!prismatic,"Internal error: expecting prismatic joint");
				// todo: find a better way of handling different joint types
                Eigen::Vector3f axis = prismatic->getJointTranslationDirection(coordSystem);

				// if necessary calculate the position part of the Jacobian
				if (mode & IKSolver::Position)
					position.block(0,i,3,1) = axis;
				// no orientation part required with prismatic joints
			}
		}

#ifdef CHECK_PERFORMANCE
        clock_t endT = clock();
        float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        if (diffClock>0.0f)
            cout << "Jacobi Loop " << i << ": RobotNode: " << dof->getName() << ", time:" << diffClock << endl;
#endif

	}
#ifdef CHECK_PERFORMANCE
        clock_t startT = clock();
#endif
	// obtain the size of the matrix.
	unsigned int size=0;
	if (mode & IKSolver::X) size++;
	if (mode & IKSolver::Y) size++;
	if (mode & IKSolver::Z) size++;
	if (mode & IKSolver::Orientation) size +=3;
	MatrixXf result(size,nDoF);

	// copy only what is required (and was previously calculated)
	unsigned int index = 0;
	if (mode & IKSolver::X) 
	{
		result.row(index) = position.row(0);
		index++;
	}
	if (mode & IKSolver::Y) 
	{
		result.row(index) = position.row(1);
		index++;
	}
	if (mode & IKSolver::Z) 
	{
		result.row(index) = position.row(2);
		index++;
	}
	if (mode & IKSolver::Orientation)
	       result.block(index,0,3,nDoF) = orientation;	

	//cout << "partial JACOBIAN: (row 1-3)" << endl;
	//cout << result.block(0,0,3,3) << endl;

#ifdef CHECK_PERFORMANCE
        clock_t endT = clock();
        float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        if (diffClock>1.0f)
            cout << "Jacobirest time:" << diffClock << endl;
#endif
	return result;
};


Eigen::MatrixXf DifferentialIK::getPseudoInverseJacobianMatrix()
{
	return getPseudoInverseJacobianMatrix(RobotNodePtr());
}
Eigen::MatrixXf DifferentialIK::getPseudoInverseJacobianMatrix(IKSolver::CartesianSelection mode)
{
	return getPseudoInverseJacobianMatrix(RobotNodePtr(),mode);
}

Eigen::MatrixXf DifferentialIK::getPseudoInverseJacobianMatrix(RobotNodePtr tcp, IKSolver::CartesianSelection mode)
{
#ifdef CHECK_PERFORMANCE
        clock_t startT = clock();
#endif
	MatrixXf Jacobian = this->getJacobianMatrix(tcp,mode);
#ifdef CHECK_PERFORMANCE
        clock_t startT2 = clock();
#endif
    Eigen::MatrixXf res = computePseudoInverseJacobianMatrix(Jacobian);
#ifdef CHECK_PERFORMANCE
        clock_t endT = clock();
        float diffClock1 = (float)(((float)(startT2 - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        float diffClock2 = (float)(((float)(endT - startT2) / (float)CLOCKS_PER_SEC) * 1000.0f);
        cout << "getPseudoInverseJacobianMatrix time1:" << diffClock1 << ", time2: " << diffClock2 << endl;
#endif
    return res;
}


void DifferentialIK::setNRows() 
{
	this->nRows=0;
	for (size_t i=0; i<tcp_set.size();i++){
		RobotNodePtr tcp = tcp_set[i];
		if (this->modes[tcp] & IKSolver::X) this->nRows++;
		if (this->modes[tcp] & IKSolver::Y) this->nRows++;
		if (this->modes[tcp] & IKSolver::Z) this->nRows++;
		if (this->modes[tcp] & IKSolver::Orientation) this->nRows += 3;
	}
};

void DifferentialIK::setGoal(const Eigen::Vector3f &goal, RobotNodePtr tcp, IKSolver::CartesianSelection mode, float tolerancePosition, float toleranceRotation){
	Matrix4f trafo;
	trafo.setIdentity();
	trafo.block(0,3,3,1)=goal;
	this->setGoal(trafo,tcp,mode,tolerancePosition,toleranceRotation);
}

RobotNodePtr DifferentialIK::getDefaultTCP()
{
	return rns->getTCP();
}

Eigen::VectorXf DifferentialIK::getDeltaToGoal(RobotNodePtr tcp)
{	
	if (!tcp)
		tcp = getDefaultTCP();
	VR_ASSERT(tcp);
	IKSolver::CartesianSelection mode = this->modes[tcp];
	return getDelta(tcp->getGlobalPose(),this->targets[tcp],this->modes[tcp]);
}


Eigen::VectorXf DifferentialIK::getDelta(Eigen::Matrix4f &current, Eigen::Matrix4f &goal, IKSolver::CartesianSelection mode)
{
	Eigen::VectorXf result(6);
	result.setZero();
 
	Vector3f position = goal.block(0,3,3,1) - current.block(0,3,3,1);
	if (mode & IKSolver::X) 
	{
		result(0) = position(0); 
	}
	if (mode & IKSolver::Y) 
	{
		result(1) = position(1); 
	}
	if (mode & IKSolver::Z) 
	{
		result(2) = position(2); 
	}
	if (mode & IKSolver::Orientation)
	{
		Matrix4f orientation = goal * current.inverse();
		AngleAxis<float> aa(orientation.block<3,3>(0,0));
		// TODO: make sure that angle is >0!?
		result.tail(3) = aa.axis()*aa.angle();	
	}

	return result;
}

VectorXf DifferentialIK::computeStep(float stepSize )
{

	if (nRows==0) this->setNRows();
	size_t nDoF = nodes.size();

	MatrixXf Jacobian(nRows,nDoF);
	VectorXf error(nRows);

	size_t index=0;
	for (size_t i=0; i<tcp_set.size();i++)
	{
		RobotNodePtr tcp = tcp_set[i];
		if (this->targets.find(tcp)!=this->targets.end())
		{
			Eigen::VectorXf delta = getDeltaToGoal(tcp);
			IKSolver::CartesianSelection mode = this->modes[tcp];
			MatrixXf partJacobian = this->getJacobianMatrix(tcp,mode);
			Jacobian.block(index,0,partJacobian.rows(),nDoF) = partJacobian;
			Vector3f position = delta.head(3);
			position *=stepSize;
			if (mode & IKSolver::X) 
			{
				error(index) = position(0); 
				index++;
			}
			if (mode & IKSolver::Y) 
			{
				error(index) = position(1);
				index++;
			}
			if (mode & IKSolver::Z) 
			{
				error(index) = position(2);
				index++;
			}
			if (mode & IKSolver::Orientation)
			{
				error.segment(index,3) = delta.tail(3)*stepSize;
				index+=3;
			}

		}
		else
			VR_ERROR << "Internal error?!" << endl; // Error


	}
	//cout << "ERROR:" << endl;
	//cout << error << endl;
	VectorXf dTheta(nDoF);

	//MatrixXf pseudo = (Jacobian.transpose() * Jacobian).inverse() * Jacobian.transpose();
	//MatrixXf pseudo = Jacobian.transpose() * (Jacobian*Jacobian.transpose()).inverse();
	MatrixXf pseudo = computePseudoInverseJacobianMatrix(Jacobian);
	//cout << "PSEUDO:" << endl;
	//cout << pseudo << endl;

	dTheta = pseudo * error;
	//cout << "THETA:" << endl;
	//cout << dTheta << endl;
	return dTheta;
	
}

float DifferentialIK::getErrorPosition(RobotNodePtr tcp)
{
	if (modes[tcp] == IKSolver::Orientation)
		return 0.0f; // ignoring position

	if (!tcp)
		tcp = getDefaultTCP();
	Vector3f position = targets[tcp].block(0,3,3,1) - tcp->getGlobalPose().block(0,3,3,1);
	//cout << "Error Position <" << tcp->getName() << ">: " << position.norm() << endl;
	//cout << boost::format("Error Position < %1% >: %2%") % tcp->getName() % position.norm()  << std::endl;
	float result = 0.0f;

	if (modes[tcp] & IKSolver::X)
		result += position(0) * position(0);
	if (modes[tcp] & IKSolver::Y)
		result += position(1) * position(1);
	if (modes[tcp] & IKSolver::Z)
		result += position(2) * position(2);

	return sqrtf(result);
}

float DifferentialIK::getErrorRotation(RobotNodePtr tcp)
{
	if (!(modes[tcp] & IKSolver::Orientation))
		return 0.0f; // no error in this dimensions
	if (!tcp)
		tcp = getDefaultTCP();
	Matrix4f orientation = this->targets[tcp] * tcp->getGlobalPose().inverse();
	AngleAxis<float> aa(orientation.block<3,3>(0,0));
	return aa.angle();	
}

float DifferentialIK::getMeanErrorPosition()
{
	if (tcp_set.size() == 0)
		return 0.0f;
	float res = 0;
	for (size_t i=0; i<tcp_set.size();i++){
		RobotNodePtr tcp = tcp_set[i];
		res += getErrorPosition(tcp);
	}
	res /= float(tcp_set.size());
	return res;
}


bool DifferentialIK::checkTolerances()
{
	bool result = true;
	for (size_t i=0; i<tcp_set.size();i++){
		RobotNodePtr tcp = tcp_set[i];
		if (getErrorPosition(tcp) > tolerancePosition[tcp] || getErrorRotation(tcp)>toleranceRotation[tcp])
		{
			result = false;
			//break;
		}
	}
	return result;
}

void DifferentialIK::checkImprovements( bool enable )
{
	checkImprovement = enable;
}

bool DifferentialIK::computeSteps(float stepSize, float minumChange, int maxNStep)
{
	VR_ASSERT(rns);
	VR_ASSERT(nodes.size() == rns->getSize());
	//std::vector<RobotNodePtr> rn = this->nodes;
	RobotPtr robot = rns->getRobot();
	VR_ASSERT(robot);
	std::vector<float> jv(nodes.size(),0.0f);
	std::vector<float> jvBest = rns->getJointValues();
	int step = 0;
	checkTolerances();
	float lastDist = FLT_MAX;

	while (step<maxNStep)
	{
		VectorXf dTheta = this->computeStep(stepSize);
		
		for (unsigned int i=0; i<nodes.size();i++)
        {
			jv[i] = (nodes[i]->getJointValue() + dTheta[i]);
			if (boost::math::isnan(jv[i]) || boost::math::isinf(jv[i]))
			{
				VR_WARNING << "Aborting, invalid joint value (nan)" << endl;
				return false;
			}
            //nodes[i]->setJointValue(nodes[i]->getJointValue() + dTheta[i]);
		}
		
		robot->setJointValues(rns,jv);
		
		// check tolerances
		if (checkTolerances())
		{
			if (verbose)
				VR_INFO << "Tolerances ok, loop:" << step << endl;
			return true;
		}
		float d = dTheta.norm();
		if (dTheta.norm()<minumChange)
		{
			if (verbose)
				VR_INFO << "Could not improve result any more (dTheta.norm()=" << d << "), loop:" << step << endl;
			return false;
		}
		float posDist = getMeanErrorPosition();
		if (checkImprovement && posDist>lastDist)
		{
			if (verbose)
				VR_INFO << "Could not improve result any more (current position error=" << posDist << ", last loop's error:" << lastDist << "), loop:" << step << endl;
			robot->setJointValues(rns,jvBest);
			return false;
		}
		jvBest = jv;
		lastDist = posDist;
		step++;
	}
	if (verbose)
	{
		VR_INFO << "IK failed, loop:" << step << endl;
		VR_INFO << "pos error:" << getErrorPosition() << endl;
		VR_INFO << "rot error:" << getErrorRotation() << endl;
	}
	return false;
}

bool DifferentialIK::solveIK( float stepSize /*= 0.2f*/, float minChange /*= 0.0f */, int maxSteps /*= 50*/ )
{
	return computeSteps(stepSize,minChange,maxSteps);
}

void DifferentialIK::convertModelScalingtoM( bool enable )
{
	convertMMtoM = enable;
}

void DifferentialIK::setVerbose( bool enable )
{
	verbose = enable;
}

} // namespace VirtualRobot
