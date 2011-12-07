
#include "CDManager.h"

#include <iostream>
#include <set>
#include <float.h>
#include "../Robot.h"


using namespace std;

//#define CCM_DEBUG

namespace VirtualRobot {

CDManager::CDManager(CollisionCheckerPtr colChecker)
{
	if (colChecker==NULL)
		this->colChecker = VirtualRobot::CollisionChecker::getGlobalCollisionChecker();
	else
		this->colChecker = colChecker;
}

CDManager::~CDManager()
{
}

void CDManager::addCollisionModel(SceneObjectSetPtr m)
{
	if (m)
	{
		colModels.push_back(m);
		if (m->getCollisionChecker() != colChecker)
		{
			VR_WARNING << "CollisionModel is linked to different instance of collision checker..." << endl;
		}
	}
}

void CDManager::addCollisionModel(SceneObjectPtr m)
{
	if (m)
	{
		if (!singleCollisionModels)
		{
			singleCollisionModels.reset(new VirtualRobot::SceneObjectSet("",colChecker));
			addCollisionModel(singleCollisionModels);
		}
		singleCollisionModels->addSceneObject(m);
		if (m->getCollisionChecker() != colChecker)
		{
			VR_WARNING << "Warning: CollisionModel is linked to different instance of collision checker..." << endl;
		}
	}
}

bool CDManager::isInCollision(SceneObjectSetPtr m)
{
	if (!m || !colChecker)
	{
		VR_WARNING << " NULL data..." << endl;
		return false;
	}

	// check all colmodels
	for (unsigned int i=0; i<colModels.size(); i++)
	{
		if (m!=colModels[i])
		{
			if (colChecker->checkCollision(colModels[i],m))
				return true;
		}
	}

	// if here -> no collision
	return false;
}


float CDManager::getDistance(SceneObjectSetPtr m)
{
	float minDist = FLT_MAX;
	float tmp;
	if (!m || !colChecker)
	{
		VR_WARNING << " NULL data..." << endl;
		return 0.0f;
	}

	// check all colmodels
	for (unsigned int i=0; i<colModels.size(); i++)
	{
		if (m!=colModels[i])
		{
			tmp = (float)colChecker->calculateDistance(colModels[i],m);
			if (tmp<minDist)
				minDist = tmp;
		}
	}

	return minDist;
}


float CDManager::getDistance()
{
	float minDist = FLT_MAX;
	float tmp;

	if (!colChecker)
		return -1.0f;

	// check all colmodels
	for (unsigned int i=0; i<colModels.size(); i++)
	{
		for (unsigned int j=i+1; j<colModels.size(); j++)
		{
			tmp = (float)colChecker->calculateDistance(colModels[i],colModels[j]);
			if (tmp<minDist)
				minDist = tmp;
		}
	}

	return minDist;
}

float CDManager::getDistance(Eigen::Vector3f &P1, Eigen::Vector3f &P2, int &trID1, int &trID2)
{
	float minDist = FLT_MAX;
	float tmp;
	Eigen::Vector3f _P1;
	Eigen::Vector3f _P2;
	int _trID1;
	int _trID2;

	if (!colChecker)
		return -1.0f;

	// check all colmodels
	for (unsigned int i=0; i<colModels.size(); i++)
	{
		for (unsigned int j=i+1; j<colModels.size(); j++)
		{
			tmp = (float)colChecker->calculateDistance(colModels[i],colModels[j],_P1,_P2,&_trID1,&_trID2);
			if (tmp<minDist)
			{
				minDist = tmp;
				trID1=_trID1;
				P1[0] = _P1[0];
				P1[1] = _P1[1];
				P1[2] = _P1[2];
				trID2=_trID2;
				P2[0] = _P2[0];
				P2[1] = _P1[1];
				P2[2] = _P2[2];
			}
		}
	}

	return minDist;
}

float CDManager::getDistance(SceneObjectSetPtr m, Eigen::Vector3f &P1, Eigen::Vector3f &P2, int &trID1, int &trID2)
{
	float minDist = FLT_MAX;
	float tmp;
	Eigen::Vector3f _P1;
	Eigen::Vector3f _P2;
	int _trID1;
	int _trID2;

	if (!colChecker || !m)
		return -1.0f;

	for (unsigned int j=0; j<colModels.size(); j++)
	{
		tmp = (float)colChecker->calculateDistance(m,colModels[j],_P1,_P2,&_trID1,&_trID2);
		if (tmp<minDist)
		{
			minDist = tmp;
			trID1 = _trID1;
			P1[0] = _P1[0];
			P1[1] = _P1[1];
			P1[2] = _P1[2];
			trID2=_trID2;
			P2[0] = _P2[0];
			P2[1] = _P1[1];
			P2[2] = _P2[2];
		}
	}

	return minDist;
}


bool CDManager::isInCollision()
{
	if (!colChecker)
		return false;
	// check all colmodels   if any exist
	for (unsigned int i=0; i<colModels.size(); i++)
	{
		for (unsigned int j=i+1; j<colModels.size(); j++)
		{
			if (colChecker->checkCollision(colModels[i],colModels[j]))
			{
				return true;
			}
		}
	}
	return false;
}


SceneObjectSetPtr CDManager::getSingleCollisionModels()
{
	return singleCollisionModels;
}

std::vector<SceneObjectSetPtr> CDManager::getSceneObjectSets()
{
	return colModels;
}

CollisionCheckerPtr CDManager::getCollisionChecker()
{
	return colChecker;
}

}
