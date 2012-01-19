#include "PoseQualityManipulability.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;


namespace VirtualRobot
{


PoseQualityManipulability::PoseQualityManipulability(VirtualRobot::RobotNodeSetPtr rns, ManipulabilityIndexType i)
:PoseQualityMeasurement(rns), manipulabilityType(i), penJointLimits(false),convertMMtoM(true)
{
	jacobian.reset(new VirtualRobot::DifferentialIK(rns,rns->getTCP()));
	jacobian->convertModelScalingtoM(convertMMtoM);
}


PoseQualityManipulability::~PoseQualityManipulability()
{
}


Eigen::MatrixXf PoseQualityManipulability::getSingularVectorCartesian()
{
	if (verbose)
		cout << "*** PoseQualityManipulability::getSingularVectorCartesian()\n";
	Eigen::MatrixXf jac = jacobian->getJacobianMatrix(rns->getTCP());
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXf U = svd.matrixU();
	Eigen::VectorXf sv = svd.singularValues();
	if (verbose)
		cout << "U:\n" << U << endl;
	if (sv.rows() == 0)
		return U;
	if (verbose)
		cout << "sv:\n" << sv << endl;
	float maxEV = sv(0);
	if (verbose)
		cout << "maxEV:" << maxEV << endl;
	for (int i=0;i<sv.rows();i++)
	{
		Eigen::MatrixXf Bl = U.block(0,i,U.rows(),1);
		Bl *= sv(i) / maxEV;
		U.block(0,i,U.rows(),1) = Bl;
	}
	if (verbose)
		cout << "result:\n" << U << endl;
	return U;
}


float PoseQualityManipulability::getManipulability(ManipulabilityIndexType i)
{
	return getManipulability(i,-1);
}

float PoseQualityManipulability::getManipulability( ManipulabilityIndexType i, int considerFirstSV )
{
	Eigen::VectorXf sv = getSingularValues();
	if (considerFirstSV<=0 || considerFirstSV>sv.rows())
		considerFirstSV = sv.rows();

	float result = 0.0f;
	switch (i)
	{
	case eMultiplySV:
		{
			if (sv.rows()>=1)
			{
				result = sv(0);
				for (int j=1;j<considerFirstSV;j++)
					result *= sv(j);
			}
			break;
		}
	case eMinMaxRatio:
		{
			if (sv.rows()>=2)
			{
				float maxSV = sv(0);
				float minSV = sv(considerFirstSV-1);
				if (maxSV!=0)
					result = minSV / maxSV;
			}
			break;
		}
	default:
		VR_ERROR << "Manipulability type not implemented..." << endl;
	}

	if (penJointLimits)
	{
		result *= getJointLimitPenalizationFactor();
	}

	return result;
}

float PoseQualityManipulability::getPoseQuality()
{
	return getManipulability(manipulabilityType);
}

Eigen::VectorXf PoseQualityManipulability::getSingularValues()
{
	Eigen::MatrixXf jac = jacobian->getJacobianMatrix(rns->getTCP());
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
	return svd.singularValues();
}

void PoseQualityManipulability::penalizeJointLimits( bool enable, float k /*= 50.0f*/ )
{
	penJointLimits = enable;
	penJointLimits_k = k;
}

float PoseQualityManipulability::getJointLimitPenalizationFactor()
{
	VR_ASSERT(rns);
	float p = 1.0f;
	for (unsigned int i=0; i<rns->getSize(); i++)
	{
		RobotNodePtr r = rns->getNode(i);
		VR_ASSERT(r);
		float d = r->getJointLimitHi() -r->getJointLimitLo();
		d = d*d;
		float a = (r->getJointValue()-r->getJointLimitLo()) * (r->getJointLimitHi() - r->getJointValue());
		if (d!=0)
			p *= a/d;
	}
	float result = 1.0f-exp(-penJointLimits_k*p);
	cout << "Pen factor:" << result << endl;
	return result;
}

}
