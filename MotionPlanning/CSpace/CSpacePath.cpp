
#include "CSpacePath.h"
#include <algorithm>
#include <iostream>
#include <fstream>
using namespace std;

namespace Saba
{

CSpacePath::CSpacePath(CSpacePtr cspace)
{
	this->cspace = cspace;
	if (!cspace)
		THROW_SABA_EXCEPTION ("No cpsace...");
		
	dimension = cspace->getDimension();
}

CSpacePath::~CSpacePath()
{
	reset();
}

void CSpacePath::reset()
{
	path.clear();
}

void CSpacePath::addPathPoint(const Eigen::VectorXf &c)
{
	SABA_ASSERT (c.rows() == dimension)
	path.push_back(c);
}

unsigned int CSpacePath::getNrOfPathPoints() const
{
	return (unsigned int)path.size();
}


Eigen::VectorXf CSpacePath::getPathEntry( unsigned int nr ) const
{
	if (nr>=path.size())
	{
		SABA_ERROR << "CSpacePath::getPathEntry: " << nr << " >= " << (unsigned int)path.size() << std::endl;
		if (path.size()>0)
			return path[path.size()-1];
		else 
		{
			Eigen::VectorXf x(dimension);
			x.setZero(dimension);
			return x;
		}
	}
	return path[nr];
}

CSpacePathPtr CSpacePath::clone() const
{
	CSpacePathPtr res(new CSpacePath(cspace));
	for (unsigned int i=0;i<getNrOfPathPoints();i++)
	{
		res->addPathPoint(getPathEntry(i));
	}
	return res;
}

CSpacePathPtr CSpacePath::createSubPath(unsigned int startIndex, unsigned int endIndex) const
{
	if (startIndex>=getNrOfPathPoints() || endIndex>=getNrOfPathPoints())
	{
		SABA_ERROR << "CSpacePath::createSubPath: wrong start or end pos" << std::endl;
		return CSpacePathPtr();
	}
	CSpacePathPtr res(new CSpacePath(cspace));
	for (unsigned int i=startIndex;i<=endIndex;i++)
	{
		res->addPathPoint(getPathEntry(i));
	}
	return res;
}

/*
bool CSpacePath::movePosition(unsigned int pos, const Eigen::VectorXf &moveVector, int sideSteps)
{
	if (pos>=getNrOfPathPoints())
	{
		SABA_ERROR << "CSpacePath::movePosition: wrong pos" << std::endl;
		return false;
	}
	if (moveVector==NULL)
	{
		SABA_ERROR << "CSpacePath::movePosition: NULL moveVec" << std::endl;
		return false;
	}

	Eigen::VectorXf c = getPathEntry(pos);

	for (unsigned int i=0;i<dimension;i++)
	{
		c[i] += moveVector[i];
	}
/*
	// apply part of moveVector to sideSteps positions next to pos
	if (sideSteps>0 && dimension>0)
	{
		float *tmp = new float[dimension];
		memcpy (tmp,moveVector,sizeof(float)*dimension);
		int posA,posB;
		float *cA,*cB;

		for (int j=1; j<=sideSteps; j++)
		{
			posA = pos - j;
			posB = pos + j;

			// half moveVector
			for (unsigned int i=0;i<dimension;i++)
			{
				tmp[i] *= 0.5f;
			}

			// left
			if (posA>=0)
			{
				cA = GetPathEntry(posA);
				for (unsigned int i=0;i<dimension;i++)
				{
					cA[i] += tmp[i];
				}
			}

			// right
			if (posB<(int)getNrOfPathPoints())
			{
				cB = GetPathEntry(posB);
				for (unsigned int i=0;i<dimension;i++)
				{
					cB[i] += tmp[i];
				}
			}
			

		} // j
		delete[] tmp;
	} // sidesteps
*/



float CSpacePath::getPathLength(bool useMetricWeights) const
{
	return getPathLength(0,(int)path.size()-1, useMetricWeights);
}

// be careful, this method calculates the c-space length of the path, not the workspace length!
float CSpacePath::getPathLength(unsigned int startIndex, unsigned int endIndex, bool forceDisablingMetricWeights) const
{
	if (endIndex<startIndex || endIndex>=path.size())
	{
		SABA_ERROR << "CSpacePath::getPathLength: wrong index..." << std::endl;
		return 0.0f;
	}
	float pathLength = 0.0f;
	Eigen::VectorXf c1,c2;
	float l;
	for (unsigned int i=startIndex;i<endIndex;i++)
	{
		c1 = path[i];
		c2 = path[i+1];

		l = cspace->calcDist(c1, c2, forceDisablingMetricWeights);
		pathLength += l;
	}
	return pathLength;
}

bool CSpacePath::getPathEntries(unsigned int start, unsigned int end , std::vector<Eigen::VectorXf> &storePosList) const
{
	if (start > end || end>=path.size())
	{
		SABA_ERROR << "CSpacePath::getPathEntries: wrong start or end.." << std::endl;
		return false;
	}
	unsigned int i = start;
	Eigen::VectorXf  data;
	while (i<=end)
	{
		data = getPathEntry(i);
		storePosList.push_back(data);
		i++;
	}
	return true;
}


void CSpacePath::erasePosition(unsigned int pos)
{
	if (pos>=path.size())
	{
		SABA_ERROR << "CSpacePath::erasePosition: pos not valid ?!" << std::endl;
		return;
	}

	std::vector<Eigen::VectorXf>::iterator iter = path.begin();
	iter += pos;

	path.erase( iter );
}


unsigned int CSpacePath::removePositions(unsigned int startPos, unsigned int endPos)
{
	if (startPos>=path.size() || endPos>=path.size() )
	{
		SABA_ERROR << "CSpacePath::removePositions: pos not valid ?!" << std::endl;
		return 0;
	}
	if (startPos>endPos)
	  return 0;

	std::vector<Eigen::VectorXf>::iterator iter = path.begin();
	iter += startPos;
	unsigned int result = 0;
	for (unsigned int i=startPos; i<=endPos; i++)
	{
		if (iter==path.end())
		{
			SABA_ERROR << "Internal error in CSpacePath::removePositions..." << std::endl;
			return result;
		}
	    //delete *iter;
	    iter = path.erase( iter );
	    result++;
	  }
	return result;
}

// inserts at position before pos
void CSpacePath::insertPosition(unsigned int pos, const Eigen::VectorXf &c)
{	
	if (pos > path.size())
	{
		std::cout << "CSpacePath::insertPosition: pos not valid ?!" << std::endl;
		return;
	}

	// copy config and insert it
	std::vector<Eigen::VectorXf>::iterator iter = path.begin();
	iter += pos;

	path.insert( iter,  c );
}

void CSpacePath::insertPosition(unsigned int pos, std::vector<Eigen::VectorXf> &newConfigurations)
{
	std::vector<Eigen::VectorXf>::iterator iter = newConfigurations.begin();
	while (iter!=newConfigurations.end())
	{
		insertPosition(pos,*iter);
		pos++;
		iter++;
	}
}

void CSpacePath::insertPath(unsigned int pos, CSpacePathPtr pathToInsert)
{
	if (!pathToInsert)
	{
		SABA_ERROR << "null data" << endl;
		return;
	}
	int i = pathToInsert->getNrOfPathPoints()-1;
	while (i >= 0)
	{
		insertPosition(pos,pathToInsert->getPathEntry(i));
		i--;
	}
}
/*
int CSpacePath::checkIntermediatePositions(unsigned int startPos, const float *samplingDist)
{

	unsigned int i = 0;
	if (startPos>= path.size()-1)
	{
		std::cout << "CSpacePath::checkIntermediatePositions: wrong position" << std::endl;
		return 0;
	}
	int nrOfNewPos = 0;
	
	// compute difference of each dimension
	float *diff = new float[dimension];
	MathHelpers::calcDiff(path[startPos],path[startPos+1],diff,dimension);
	
	// compute intermediate step for each dimension, search for maximum step in all dimensions
	int *step = new int[dimension];
	int maxStep = -1;
	bool intermediateStepsNecessary = false;
	for (i=0;i<dimension;i++)
	{
		if (fabs(diff[i])>samplingDist[i]) intermediateStepsNecessary = true; // intermediate steps necessary!
		step[i] =(int)(floor(fabs(diff[i] / samplingDist[i])));
		if (maxStep<step[i]) maxStep = step[i];
	}
	
	float *actPos = new float[dimension];
	float *goalPos = new float[dimension];
	memcpy(actPos,path[startPos],sizeof(float)*dimension); // copy start pos
	memcpy(goalPos,path[startPos+1],sizeof(float)*dimension); // copy goal pos
	
	if (intermediateStepsNecessary)
	{
		// apply these maxSteps and create new path configurations except for path[startPos+1]
		for (i=1;(int)i<maxStep;i++)
		{
			float factor = (float)i / (float)maxStep;
			for (unsigned int j=0;j<dimension;j++)
			{
				actPos[j] = path[startPos][j] + (goalPos[j] - path[startPos][j]) * factor;// go on one step
			}
			insertPosition(startPos+i,actPos);
			nrOfNewPos++;
		}
	}
		
	// finally create a new path configuration to hold path[startPos+1]
	for (unsigned int j=0;j<dimension;j++)
	{
		actPos[j] = goalPos[j]; // go on one step
	}
	insertPosition(startPos+i,actPos);
	nrOfNewPos++;
	
	delete diff;
	delete step;
	delete actPos;
	delete goalPos;
	
	return nrOfNewPos;
}

// returns number of added nodes
int CSpacePath::checkIntermediatePositions(unsigned int startPos, float samplingDist, CSpaceSampled *cspace)
{
	if (cspace==NULL)
		return 0;

	float dist = 0.0f;
	unsigned int i;
	int newConfigs = 0;
	dist = cspace->getDist(path[startPos],path[startPos+1]);//MathHelpers::calcWeightedDist(startNode->configuration,config,m_metricWeights,m_nTransDim,m_nRotDim,dimension);
	if (dist==0.0f)
	{
		// nothing to do
		//std::cout << "append path: zero dist to new config!" << std::endl;
		return 0;
	}

	float *lastConfig = path[startPos];
	float *config = path[startPos+1];
	float *actPos = new float[dimension];

	while (dist>samplingDist)
	{
		float factor = samplingDist / dist;
		// create a new node with config, store it in nodeList and set parentID
	
		// copy values
		for (i=0; i<dimension;i++)
		{
			actPos[i] = cspace->interpolate(lastConfig,config,i,factor);// lastConfig[i] + (config[i] - lastConfig[i])*factor;
		}
		
		newConfigs++;
		insertPosition(startPos+newConfigs,actPos);
		lastConfig = actPos;
		dist = cspace->getDist(actPos,config);//MathHelpers::calcWeightedDist(newNode->configuration,config,m_metricWeights,m_nTransDim,m_nRotDim,dimension);
	}
	delete[] actPos;
	return newConfigs;
}
*/

float CSpacePath::getTime(unsigned int nr)
{
	if (getNrOfPathPoints()==0)
	{
		SABA_ERROR << " no Path.." << std::endl;
	}
	if(nr < 0 || nr > getNrOfPathPoints()-1)
	{
		SABA_ERROR << " path entry " << nr << " doesnt exist" << std::endl;
		if(nr < 0) nr = 0;
		if(nr > getNrOfPathPoints()-1) nr = getNrOfPathPoints() - 1;
	}

	float t = 0.0f;

	float l = getPathLength();

	Eigen::VectorXf c1 = getPathEntry(0);

	for(unsigned int i = 0; i < nr; i++)
	{
		Eigen::VectorXf c2 = getPathEntry(i+1);
		t += cspace->calcDist(c1,c2)/l;
		//t += MathHelpers::calcDistRotational(c1, c2, dimension, m_rotationalDimension)/l;
		c1 = c2;
	}
	
	return t;
}

// returns position on path for time t (0<=t<=1)
void CSpacePath::interpolatePath( float t, Eigen::VectorXf &storePathPos, int *storeIndex /*= NULL*/ ) const
{
	storePathPos.resize(dimension);
	if (t<0 || t>1.0f)
	{
		// check for rounding errors
		if (t<-0.000000001 || t>1.000001f)
		{
			std::cout << "CSpacePath::interpolatePath: need t value between 0 and 1... (" << t << ")" << std::endl;
		}
		if (t<0)
			t = 0.0f;
		if (t>1.0f)
			t = 1.0f;
	}

	if (getNrOfPathPoints()==0)
	{
		SABA_WARNING << "CSpacePath::interpolatePath: no Path.." << std::endl;
	}
	if (t==0.0f)
	{
		storePathPos = getPathEntry(0);
		if (storeIndex!=NULL)
			*storeIndex = 0;
		return;
	}
	else if (t==1.0f)
	{
		storePathPos = getPathEntry(getNrOfPathPoints()-1);
		if (storeIndex!=NULL)
			*storeIndex = (int)path.size();
		return;
	}


	float l = getPathLength();
	float wantedLength = l * t;
	float actLength = 0.0f;
	unsigned int startIndex = 0;
	Eigen::VectorXf c1 = getPathEntry(startIndex);
	Eigen::VectorXf c2 = c1;
	float lastLength = 0.0f;
	// search path segment for wantedLength
	while (actLength < wantedLength && startIndex<getNrOfPathPoints()-1)
	{
		c1 = c2;
		startIndex++;
		c2 = getPathEntry(startIndex);
		lastLength = cspace->calcDist(c1,c2);
		//lastLength = MathHelpers::calcDistRotational(c1, c2, dimension, m_rotationalDimension);
		actLength += lastLength;
	}
	startIndex--;
	actLength-=lastLength;

	// segment starts with startIndex
	float restLength = wantedLength - actLength;

	float factor = 0.0f;
	if (lastLength>0)
		factor = restLength / lastLength;
	if (factor>1.0f)
	{
		// ignore rounding errors
		factor = 1.0f;
	}
	for (unsigned int j=0;j<dimension;j++)
	{
		if (cspace->isBorderlessDimension(j))
		{
			float diff = (c2[j] - c1[j]);
			if (diff>M_PI)
				diff -= (float)M_PI*2.0f;
			if (diff<-M_PI)
				diff += (float)M_PI*2.0f;
			storePathPos[j] = c1[j] +  diff * factor; // storePos = startPos + factor*segment
		} else
		{
			storePathPos[j] = c1[j] + (c2[j] - c1[j]) * factor; // storePos = startPos + factor*segment
		}
	}
	if (storeIndex!=NULL)
		*storeIndex = startIndex;

}

void CSpacePath::reverse()
{
  std::reverse(path.begin(), path.end());
}

void CSpacePath::print() const
{
	std::cout << "<CSpacePath size='" << path.size() << "' dim='" << dimension << "'>" << std::endl << std::endl;
	for (unsigned int i = 0; i < path.size(); i++)
	{
		std::cout << "\t<Node id='" << i << "'>" << std::endl;
		Eigen::VectorXf c = path[i];
		for (unsigned int k = 0; k < dimension; k++)
		{
			std::cout << "\t\t<c value='" << c[k] << "'/>" << std::endl;
		}
		std::cout << "\t</Node>" << std::endl << std::endl;
	}

	std::cout << "</CSpacePath>" << std::endl;
}

const std::vector <Eigen::VectorXf>& CSpacePath::getPathData() const
{
	return path;
}

std::vector<Eigen::Matrix4f > CSpacePath::createWorkspacePath( VirtualRobot::RobotNodePtr r )
{
	VR_ASSERT(r);
	VR_ASSERT(cspace);
	VirtualRobot::RobotNodeSetPtr rns = cspace->getRobotNodeSet();
	VR_ASSERT(rns);
	std::vector<Eigen::Matrix4f > result;

	if (cspace->hasExclusiveRobotAccess())
		CSpace::lock();

	for (size_t i = 0; i < path.size(); i++)
	{
		// get tcp coords:
		rns->setJointValues(path[i]);
		Eigen::Matrix4f m;
		result.push_back(r->getGlobalPose());
	}

	if (cspace->hasExclusiveRobotAccess())
		CSpace::unlock();
	return result;
}

Saba::CSpacePtr CSpacePath::getCSpace()
{
	return cspace;
}

} // namespace Saba
