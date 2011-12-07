// **************************************************************
// Implementation of class GraspQualityMeasure
// **************************************************************
// Author: Niko Vahrenkamp
// Date: 26.10.2011
// **************************************************************


// **************************************************************
// includes
// **************************************************************

#include "GraspQualityMeasure.h"
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/SceneObject.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>

using namespace std;
using namespace VirtualRobot;

namespace GraspStudio
{


GraspQualityMeasure::GraspQualityMeasure(VirtualRobot::SceneObjectPtr object, float unitForce, float frictionConeCoeff, int frictionConeSamples )
    : object(object), unitForce(unitForce),frictionCoeff(frictionConeCoeff),frictionConeSamples(frictionConeSamples)
{
	THROW_VR_EXCEPTION_IF(!object,"Need an object");
	THROW_VR_EXCEPTION_IF(!object->getCollisionModel(),"Need an object with collision model");
	THROW_VR_EXCEPTION_IF(!object->getCollisionModel()->getTriMeshModel(),"Need an object with trimeshmodel");

	coneGenerator.reset(new ContactConeGenerator(frictionConeSamples, frictionCoeff, unitForce));

	//Member variable representing Grasp Quality ranging from 1 to 0
	graspQuality = 0.0;
	verbose = false;
	objectLength = 0.0f;

	centerOfModel = object->getCollisionModel()->getTriMeshModel()->getCOM();

	Eigen::Vector3f minS,maxS;
	object->getCollisionModel()->getTriMeshModel()->getSize(minS,maxS);
	maxS = maxS - minS;
	objectLength = maxS(0);
	if (maxS(1)>objectLength)
		objectLength = maxS(1);
	if (maxS(2)>objectLength)
		objectLength = maxS(2);
}

GraspQualityMeasure::~GraspQualityMeasure()
{
}

void GraspQualityMeasure::setContactPoints( const std::vector<VirtualRobot::MathTools::ContactPoint> &contactPoints6d )
{
	//if (contactPoints.size() < 2)
	//	return;
	this->contactPoints.clear();
	this->contactPointsM.clear();
	std::vector<MathTools::ContactPoint>::const_iterator objPointsIter;
	for (objPointsIter = contactPoints.begin(); objPointsIter != contactPoints.end(); objPointsIter++)
	{
		MathTools::ContactPoint point = (*objPointsIter);
		point.p -= centerOfModel;
		point.n.normalize();
		
		this->contactPoints.push_back(point);
	}
	MathTools::convertMM2M(this->contactPoints,this->contactPointsM);
	if (verbose)
	{
		GRASPSTUDIO_INFO << ": Nr of contact points:" << this->contactPoints.size() << endl;
	}
}

void GraspQualityMeasure::setContactPoints( const std::vector<EndEffector::ContactInfo> &contactPoints )
{
	//if (contactPoints.size() < 2)
	//	return;
	this->contactPoints.clear();
	this->contactPointsM.clear();
	std::vector<EndEffector::ContactInfo>::const_iterator objPointsIter;
	for (objPointsIter = contactPoints.begin(); objPointsIter != contactPoints.end(); objPointsIter++)
	{
		MathTools::ContactPoint point;

		point.p = objPointsIter->contactPointObstacleLocal;
		point.p -= centerOfModel;

		point.n = objPointsIter->contactPointFingerLocal - objPointsIter->contactPointObstacleLocal;
		point.n.normalize();

		this->contactPoints.push_back(point);
	}
	VirtualRobot::MathTools::convertMM2M(this->contactPoints,this->contactPointsM);

	if (verbose)
	{
		GRASPSTUDIO_INFO << ": Nr of contact points:" << this->contactPoints.size() << endl;
	}
}


bool GraspQualityMeasure::sampleObjectPoints(int nMaxFaces)
{
	sampledObjectPoints.clear();
	sampledObjectPointsM.clear();

    TriMeshModelPtr model = object->getCollisionModel()->getTriMeshModel();
	//Eigen::Vector3f _com = model->getCOM();
	MathTools::ContactPoint objectPoint;
	if (verbose)
		GRASPSTUDIO_INFO << ": object COM: << " << centerOfModel[0] << "," << centerOfModel[1] << "," << centerOfModel[2] << endl;

	int nFaces = (int)model->faces.size();
	if (nFaces > nMaxFaces)
	{
		nFaces = nMaxFaces;
	}
	int nLoopCount = 0;
	std::vector<MathTools::TriangleFace> vFaceCopy = model->faces;
	std::vector<MathTools::TriangleFace>::iterator iFaceIter;

	while (nLoopCount<nFaces && vFaceCopy.size()>0)
	{
		int nRnd = rand() % vFaceCopy.size();
		iFaceIter = vFaceCopy.begin();
		iFaceIter += nRnd;

		objectPoint.p = (model->vertices[iFaceIter->id1] + model->vertices[iFaceIter->id2] + model->vertices[iFaceIter->id3]) / 3.0f;
		objectPoint.n = iFaceIter->normal;
		objectPoint.n.normalize();
		objectPoint.n *= unitForce;
		
		// move points so that object is located at origin
		objectPoint.p -= centerOfModel;
		sampledObjectPoints.push_back(objectPoint);

		vFaceCopy.erase(iFaceIter);
		nLoopCount++;
	}

	MathTools::convertMM2M(sampledObjectPoints,sampledObjectPointsM);
	if (verbose)
	{
		GRASPSTUDIO_INFO << ": Nr of sample object points:" << sampledObjectPoints.size() << endl;
	}

	return (sampledObjectPoints.size()>0);
}


MathTools::ContactPoint GraspQualityMeasure::getContactPointsCenter()
{
	MathTools::ContactPoint p;
	p.p.setZero();
	p.n.setZero();
	if (contactPoints.size()==0)
		return p;
	for (int i=0;i<(int)contactPoints.size();i++)
	{
		p.p += contactPoints[i].p;
		p.n += contactPoints[i].n;

	}
	p.p /= (float)contactPoints.size();
	p.n /= (float)contactPoints.size();

	return p;
}

MathTools::ContactPoint GraspQualityMeasure::getSampledObjectPointsCenter()
{
	MathTools::ContactPoint p;
	p.p.setZero();
	p.n.setZero();
	if (sampledObjectPoints.size()==0)
		return p;
	for (int i=0;i<(int)sampledObjectPoints.size();i++)
	{
		p.p += sampledObjectPoints[i].p;
		p.n += sampledObjectPoints[i].n;
	}
	p.p /= (float)sampledObjectPoints.size();
	p.n /= (float)sampledObjectPoints.size();
	return p;
}

void GraspQualityMeasure::setVerbose( bool enable )
{
	verbose = enable;
}

std::string GraspQualityMeasure::getName()
{
	std::string sName("GraspQualityMeasure");
	return sName;
}

Eigen::Vector3f GraspQualityMeasure::getCoM()
{
	return centerOfModel;
}

VirtualRobot::SceneObjectPtr GraspQualityMeasure::getObject()
{
	return object;
}


} // namespace
