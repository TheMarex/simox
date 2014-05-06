#include <Eigen/Geometry>
#include "JacobiProvider.h"

#include <boost/pointer_cast.hpp>

#include <algorithm>

using namespace Eigen;

//#define CHECK_PERFORMANCE

namespace VirtualRobot
{

JacobiProvider::JacobiProvider(RobotNodeSetPtr rns, InverseJacobiMethod invJacMethod) : 
	rns(rns), inverseMethod(invJacMethod)
{	
	initialized = false;
}

JacobiProvider::~JacobiProvider()
{	
}

Eigen::MatrixXf JacobiProvider::getPseudoInverseJacobianMatrix(SceneObjectPtr tcp)
{
#ifdef CHECK_PERFORMANCE
	clock_t startT = clock();
#endif
	MatrixXf Jacobian = this->getJacobianMatrix(tcp);
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


Eigen::MatrixXf JacobiProvider::getPseudoInverseJacobianMatrix()
{
	MatrixXf Jacobian = this->getJacobianMatrix();
	return computePseudoInverseJacobianMatrix(Jacobian);
	//return getPseudoInverseJacobianMatrix(rns->getTCP());
}

Eigen::MatrixXf JacobiProvider::computePseudoInverseJacobianMatrix(const Eigen::MatrixXf &m) const
{
#ifdef CHECK_PERFORMANCE
	clock_t startT = clock();
#endif
	MatrixXf pseudo;
	switch (inverseMethod)
	{
	case eTranspose:
		{
			if (jointWeights.rows() == m.cols())
			{
				Eigen::MatrixXf W = jointWeights.asDiagonal();
				Eigen::MatrixXf W_1 = W.inverse();
				pseudo = W_1 * m.transpose() * (m*W_1*m.transpose()).inverse();
			}
			else
			{
				pseudo = m.transpose() * (m*m.transpose()).inverse();
			}
			break;
		}
	case eSVD:
		{
				 float pinvtoler = 0.00001f;
				 pseudo = MathTools::getPseudoInverse(m, pinvtoler);
				 break;
		}
	case eSVDDamped:
		{
				 float pinvtoler = 0.00001f;
				 pseudo = MathTools::getPseudoInverseDamped(m,pinvtoler);
				 break;
		}
	default:
		THROW_VR_EXCEPTION("Inverse Jacobi Method nyi...");
	}
#ifdef CHECK_PERFORMANCE
	clock_t endT = clock();
	float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
	//if (diffClock>10.0f)
	cout << "Inverse Jacobi time:" << diffClock << endl;
#endif
	return pseudo;
}

VirtualRobot::RobotNodeSetPtr JacobiProvider::getRobotNodeSet()
{
	return rns;
}

void JacobiProvider::setJointWeights(const Eigen::VectorXf &jointWeights)
{
	this->jointWeights = jointWeights;
}

bool JacobiProvider::isInitialized()
{
	return initialized;
}

} // namespace VirtualRobot
