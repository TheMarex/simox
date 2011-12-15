// **************************************************************
// Implementation of class ContactConeGenerator
// **************************************************************
// Author: Niko Vahrenkamp, Martin Do
// Date: 27.10.2011
// **************************************************************


// **************************************************************
// includes
// **************************************************************

#include "ContactConeGenerator.h"
#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

using namespace std;
using namespace VirtualRobot;

namespace GraspStudio
{

ContactConeGenerator::ContactConeGenerator(int coneSamples, float frictionCoeff, float unitForce)
{

	this->unitForce = unitForce;
	//Generation of generic friction cone discretized by an 8-sided polyhedron
	this->frictionCoeff = frictionCoeff;
	this->frictionConeAngle = atan(frictionCoeff);
	this->frictionConeRad = 2*unitForce*sin((unitForce*2*frictionConeAngle) / (2*unitForce));
	this->frictionConeSamples = coneSamples;
	for (int i = 0; i < frictionConeSamples; i++)
	{
	    Eigen::Vector3f p;
	    p(0) = unitForce*(float)(cos(frictionConeAngle)*cos(i*2.0*M_PI/frictionConeSamples));
		p(1) = unitForce*(float)(cos(frictionConeAngle)*sin(i*2.0*M_PI/frictionConeSamples));
		p(2) = unitForce*(float)cos(frictionConeAngle);
		frictionConeRimPoints.push_back(p);
	}
}

ContactConeGenerator::~ContactConeGenerator()
{
    frictionConeRimPoints.clear();
}



void ContactConeGenerator::computeConePoints( const VirtualRobot::MathTools::ContactPoint &point, std::vector<VirtualRobot::MathTools::ContactPoint> &storeConePoints )
{
	//Rotate generic friction cone to align with object normals
	
	Eigen::Vector3f upRightNormal(0.0f,0.0f,1.0f);
	MathTools::Quaternion objNormalRot =  MathTools::getRotation(upRightNormal,point.n);
    Eigen::Matrix4f objNormalTrafo = MathTools::quat2eigen4f(objNormalRot);
        
    Eigen::Vector3f conePoint;
	
	/*SbVec3f conePoint;
	SbVec3f objPoint(point.p(0),point.p(1),point.p(2));
	SbVec3f objNormal(point.n(0),point.n(1),point.n(2));
	SbRotation objNormalRot(objNormal,SbVec3f(0.0,0.0,1.0));
	SbMatrix objNormalTrafo;
	objNormalTrafo.setRotate(objNormalRot);*/

	float scaleFactor = 1.0f;
	for (int i = 0; i < frictionConeSamples; i++)
	{
		VirtualRobot::MathTools::ContactPoint newConePoint;
		Eigen::Vector3f conePointOrg = frictionConeRimPoints[i]*scaleFactor;
		conePoint = MathTools::transformPosition(conePointOrg,objNormalTrafo);
		    
		//SbVec3f conePointOrg((float)frictionConeRimPoints[i][0]*scaleFactor,(float)frictionConeRimPoints[i][1]*scaleFactor,(float)frictionConeRimPoints[i][2]*scaleFactor);
		//objNormalTrafo.multMatrixVec(conePointOrg, conePoint);
		newConePoint.p = conePoint + point.p;
		newConePoint.n = conePoint;
		newConePoint.n.normalize();
		
		/*newConePoint.p(0) = conePoint[0]+objPoint[0];
		newConePoint.p(1) = conePoint[1]+objPoint[1];
		newConePoint.p(2) = conePoint[2]+objPoint[2];
		newConePoint.n(0) = conePoint[0];
		newConePoint.n(1) = conePoint[1];
		newConePoint.n(2) = conePoint[2];*/
		/*float l = sqrtf(conePoint[0]*conePoint[0] + conePoint[1]*conePoint[1] + conePoint[2]*conePoint[2]);
		if (l>=1e-8)
		{
			l = 1.0f / l;
			newConePoint.n(0) *= l;
			newConePoint.n(1) *= l;
			newConePoint.n(2) *= l;
		}*/
		storeConePoints.push_back(newConePoint);
	}
}

void ContactConeGenerator::computeConePoints(const VirtualRobot::MathTools::ContactPoint &point, std::vector<Eigen::Vector3f> &storeConePoints)
{
    
    //Rotate generic friction cone to align with object normals
	
	Eigen::Vector3f upRightNormal(0.0f,0.0f,1.0f);
	MathTools::Quaternion objNormalRot =  MathTools::getRotation(upRightNormal,point.n); // invert?!
    Eigen::Matrix4f objNormalTrafo = MathTools::quat2eigen4f(objNormalRot);
        
    Eigen::Vector3f conePoint;

	float scaleFactor = 1.0f;
	for (int i = 0; i < frictionConeSamples; i++)
	{
		Eigen::Vector3f newConePoint;
		Eigen::Vector3f conePointOrg = frictionConeRimPoints[i]*scaleFactor;
		conePoint = MathTools::transformPosition(conePointOrg,objNormalTrafo);
		newConePoint = conePoint + point.p;
		storeConePoints.push_back(newConePoint);
	}
    /*
	//Rotate generic friction cone to align with object normals
	SbVec3f conePoint;
	SbVec3f objPoint(point.p(0),point.p(1),point.p(2));
	SbVec3f objNormal(point.n(0),point.n(1),point.n(2));
	SbRotation objNormalRot(objNormal,SbVec3f(0.0,0.0,1.0));
	SbMatrix objNormalTrafo;
	objNormalTrafo.setRotate(objNormalRot);
	float scaleFactor = 1.0f;
	for (int i = 0; i < frictionConeSamples; i++)
	{
		GraspStudio::Vec3D newConePoint;
		SbVec3f conePointOrg((float)frictionConeRimPoints[i][0]*scaleFactor,(float)frictionConeRimPoints[i][1]*scaleFactor,(float)frictionConeRimPoints[i][2]*scaleFactor);
		objNormalTrafo.multMatrixVec(conePointOrg, conePoint);
		newConePoint.p(0) = conePoint[0]+objPoint[0];
		newConePoint.p(1) = conePoint[1]+objPoint[1];
		newConePoint.p(2) = conePoint[2]+objPoint[2];
		storeConePoints.push_back(newConePoint);
	}*/
}

} // namespace

