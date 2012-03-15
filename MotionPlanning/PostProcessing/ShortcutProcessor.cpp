
#include "ShortcutProcessor.h"
#include "CSpace/CSpaceSampled.h"
#include "CSpace/CSpacePath.h"
#include <vector>
#include <time.h>
#include <math.h>

namespace Saba
{

ShortcutProcessor::ShortcutProcessor(CSpacePathPtr path, CSpaceSampledPtr cspace, bool verbose) : PathProcessor(path, verbose), cspace(cspace)
{
	stopOptimization = false;
}

ShortcutProcessor::~ShortcutProcessor()
{
}



int ShortcutProcessor::tryRandomShortcut(int maxSolutionPathDist)
{
    if (!path || !cspace)
    {
        SABA_ERROR << "NULL data" << endl;       
        return 0;
    }

	if (maxSolutionPathDist<2)
		maxSolutionPathDist = 2;

	if (verbose)
	{
		SABA_INFO << "Path length: " << optimizedPath->getLength() << std::endl;
		SABA_INFO << "Path nr of nodes: " << optimizedPath->getNrOfPoints() << std::endl;
	}
	int startNodeIndex = (int)(rand()%(optimizedPath->getNrOfPoints()-1));

	int dist = rand()%(2*maxSolutionPathDist)-maxSolutionPathDist;
	if (dist==0)
		dist++;
	if (dist==1)
		dist++;
	if (dist==-1)
		dist--;

	int endNodeIndex = startNodeIndex+dist;
	if (startNodeIndex>endNodeIndex)
	{
		int tmp = startNodeIndex;
		startNodeIndex = endNodeIndex;
		endNodeIndex = tmp;
	}

	if (startNodeIndex<0)
		startNodeIndex = 0;
	if (endNodeIndex>(int)optimizedPath->getNrOfPoints()-1) // last node should remain unchanged 
		endNodeIndex = (int)optimizedPath->getNrOfPoints()-1;
	if ((endNodeIndex-startNodeIndex)<=1)
		return 0;

	if (verbose)
		SABA_INFO << "-- start: " << startNodeIndex << ", end: " << endNodeIndex << std::endl;

	Eigen::VectorXf s = optimizedPath->getPoint(startNodeIndex);
	Eigen::VectorXf e = optimizedPath->getPoint(endNodeIndex);
	Eigen::VectorXf d = e - s;

	// test line between start and end

	float distShortcut = d.norm();
	float distPath = optimizedPath->getLength(startNodeIndex,endNodeIndex);

	// -------------------------------------------------------------------
	// DEBUG
	if (verbose)
		std::cout << "-- distShortcut: " << distShortcut << " distPath: " << distPath << std::endl;
	// -------------------------------------------------------------------

	if (distShortcut<distPath*0.99f)
	{
		// -------------------------------------------------------------------
		// DEBUG
		if (verbose)
			std::cout << "ShortcutProcessor: tryRandomShortcut: Shortcut Path shorter!" << std::endl;
		// -------------------------------------------------------------------

		//bool pathOK = m_pTree->checkPathSampled(startConfig,endConfig);
		// first check sampled, the validate with freeBubbles 
		// (if sampled check failed we don't need to do the expensive freeBubble check)
		//bool pathOK = m_pTree->checkPathSampled(startConfig,endConfig);
		bool pathOK = cspace->isPathValid(s,e);
		if (pathOK)
		{
			/*cout << "start:" << endl << s << endl;
			cout << "end:" << endl << e << endl;
			cout << "all:" << endl;
			optimizedPath->print();*/
			// -------------------------------------------------------------------
			// DEBUG
			if (verbose)
				std::cout << "ShortcutProcessor::tryRandomShortcut: pathOK!" << std::endl;
			// -------------------------------------------------------------------

			// complete path valid and dist is shorter
			if (verbose)
				std::cout << "Creating direct shortcut from node " << startNodeIndex << " to node " << endNodeIndex << std::endl;
			for (int i=endNodeIndex-1; i>=startNodeIndex+1;i--)
			{
				// erase solution positions
				optimizedPath->erasePosition(i);
			}
			if (verbose)
			{
				float distPathtest = optimizedPath->getLength(startNodeIndex,startNodeIndex+1);
				std::cout << "-- erased intermediate positions, distPath startIndex to (startIndex+1): " << distPathtest << std::endl;
			}
			/*cout << "all2:" << endl;
			optimizedPath->print();*/
			// create intermediate path		

			CSpacePathPtr intermediatePath = cspace->createPath(s, e);
			int newP = 0;
			if (intermediatePath->getNrOfPoints()>2)
			{
				newP = intermediatePath->getNrOfPoints()-2;
				/*cout << "before:" << endl;
				optimizedPath->print();
				cout << "interm path:" << endl;
				intermediatePath->print();*/
				intermediatePath->erasePosition(intermediatePath->getNrOfPoints()-1);
				intermediatePath->erasePosition(0);
				/*cout << "interm path without start end:" << endl;
				intermediatePath->print();*/
				optimizedPath->insertTrajectory(startNodeIndex+1,intermediatePath);
				/*cout << "after:" << endl;
				optimizedPath->print();	*/
			}
		
			if (verbose)
			{
				float sum = 0.0f;
				for (int u=startNodeIndex; u<=startNodeIndex+newP; u++)
				{
					float distPathtest2 = optimizedPath->getLength(u,u+1);
					sum += distPathtest2;
					std::cout << "---- intermediate position: " << u << ", distPath to next pos: " << distPathtest2 << ", sum:" << sum << std::endl;
				}
			}
			int nodes = endNodeIndex-startNodeIndex-1 + newP;
			if (verbose)
			{
				std::cout << "-- end, nodes: " << nodes << std::endl;
			}


			return nodes;
		} 
	} // dist reduced

	// path not valid
	return 0;
}


CSpacePathPtr ShortcutProcessor::optimize(int optimizeSteps)
{
	return shortenSolutionRandom(optimizeSteps);
}

CSpacePathPtr ShortcutProcessor::shortenSolutionRandom(int shortenLoops /*=300*/, int maxSolutionPathDist)
{
	stopOptimization = false;
	THROW_VR_EXCEPTION_IF((!cspace || !path), "NULL data");	
	int counter = 0;
	optimizedPath = path->clone();
	if (!optimizedPath)
	{
		std::cout << "ShortcutProcessor::ShortenSolutionRandom: Wrong parameters or no path to smooth..." << std::endl;
		return CSpacePathPtr();
	}
	if (optimizedPath->getNrOfPoints()<=2)
	    return optimizedPath;
	    
	int result = 0;
	int beforeCount = (int)optimizedPath->getNrOfPoints();
	float beforeLength = optimizedPath->getLength();
	if (verbose)
	{
	    SABA_INFO << ": solution size before shortenSolutionRandom:" << beforeCount << std::endl;
	    SABA_INFO << ": solution length before shortenSolutionRandom:" << beforeLength << std::endl;
	}
	clock_t startT = clock();

	int red;
	int loopsOverall = 0;
	while (counter<shortenLoops && !stopOptimization)
	{
		loopsOverall++;
		red = tryRandomShortcut(maxSolutionPathDist);
		
		counter++;
		result += red;
	}
	if(stopOptimization)
	{
		SABA_INFO << "optimization was stopped" << std::endl;
	}
	int afterCount = (int)optimizedPath->getNrOfPoints();
	float afterLength = optimizedPath->getLength();
	clock_t endT = clock();
	float timems = (float)(endT - startT) / (float)CLOCKS_PER_SEC * 1000.0f;
	if (verbose)
	{
	    SABA_INFO << ": shorten loops: " << loopsOverall << std::endl;
	    SABA_INFO << ": shorten time: " << timems << " ms " << std::endl;
	    SABA_INFO << ": solution size after ShortenSolutionRandom (nr of positions) : " << afterCount << std::endl;
	    SABA_INFO << ": solution length after ShortenSolutionRandom : " << afterLength << std::endl;
    }
	return optimizedPath;
}

void ShortcutProcessor::doPathPruning()
{
	optimizedPath = path->clone();

    unsigned int i = 0;
    while (i < optimizedPath->getNrOfPoints()-2)
    {
        Eigen::VectorXf startConfig = optimizedPath->getPoint(i);
        Eigen::VectorXf endConfig = optimizedPath->getPoint(i+2);
        if(cspace->isPathValid(startConfig,endConfig) )
        {
            optimizedPath->erasePosition(i+1);
            if (i > 0) i--;
        }
        else
        {
          i++;
        }
    }
}


} // namespace
