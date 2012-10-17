/**
* @package    VirtualRobot
* @author     Manfred Kroehnert 
* @copyright  2010 Manfred Kroehnert
*/


#include "CoinVisualization.h"
#include "CoinVisualizationNode.h"

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoUnits.h>

#include <boost/foreach.hpp>
#include <algorithm>

namespace VirtualRobot {

CoinVisualization::CoinVisualization(const VisualizationNodePtr visualizationNode) :
Visualization(visualizationNode)
{
    selection = NULL;
}

CoinVisualization::CoinVisualization(const std::vector<VisualizationNodePtr>& visualizationNodes) :
Visualization(visualizationNodes)
{
    selection = NULL;
}
	
CoinVisualization::~CoinVisualization()
{
    if (selection)
        selection->unref();
}

bool CoinVisualization::buildVisualization()
{
    if (selection)
        return true;
    selection = new SoSelection;
    selection->ref();
    selection->policy = SoSelection::TOGGLE;
    SoSeparator* visualization = new SoSeparator();
	//SoMatrixTransform *mtr = new SoMatrixTransform;
	/*SbMatrix m(reinterpret_cast<SbMat*>(globalPose.data()));
	mtr->matrix.setValue(m);
	selection->addChild(mtr);*/
    SoUnits *u = new SoUnits();
    u->units = SoUnits::METERS;
    visualization->addChild(u);
	selection->addChild(visualization);

    BOOST_FOREACH(VisualizationNodePtr visualizationNode, visualizationNodes)
    {
            boost::shared_ptr<CoinVisualizationNode> coinVisualizationNode= boost::dynamic_pointer_cast<CoinVisualizationNode>(visualizationNode);
            if (coinVisualizationNode && coinVisualizationNode->getCoinVisualization())
                    visualization->addChild(coinVisualizationNode->getCoinVisualization());
    }
    return true;
}

bool CoinVisualization::highlight(unsigned int which, bool enable)
{
    if (which >= visualizationNodes.size())
    {
        VR_WARNING << "Could not find visualizationNode..." << endl;
        return false;
    }
    return highlight(visualizationNodes[which],enable);
}

bool CoinVisualization::highlight(SoNode* visu, bool enable)
{
	if (!visu)
		return false;
	if (enable)
		selection->select(visu);
	else
		selection->deselect(visu);

	selection->touch();
	return true;
}

bool CoinVisualization::highlight(VisualizationNodePtr visualizationNode, bool enable)
{
    if (!selection)
        return false;

    if (!isVisualizationNodeRegistered(visualizationNode))
    {
        VR_WARNING << "Could not find visualizationNode..." << endl;
        return false;
    }

    boost::shared_ptr<CoinVisualizationNode> coinVisualizationNode = boost::dynamic_pointer_cast<CoinVisualizationNode>(visualizationNode);


	if (coinVisualizationNode)
	{
		return highlight(coinVisualizationNode->getCoinVisualization(),enable);
	}

    return false;
}

bool CoinVisualization::highlight( bool enable )
{
	for (size_t i = 0;i<visualizationNodes.size();i++)
		highlight(i,enable);
	return true;
}

/**
 * This method iterates over the entries in member
 * CoinVisualization::visualizationNodes and stores the return value of
 * CoinVisualizationNode::getCoinVisualization() in an SoSeparator if the
 * processed node is of type CoinVisualizationNode.
 * Afterwards the SoSeparator is returned.
 */
SoNode* CoinVisualization::getCoinVisualization()
{
    buildVisualization();
    return selection;
}

/**
 * \return new instance of VirtualRobot::CoinVisualization with the same set of robot nodes.
 */
VirtualRobot::VisualizationPtr CoinVisualization::clone()
{
	return VisualizationPtr(new CoinVisualization(visualizationNodes));
}

/*void CoinVisualization::setGlobalPose( const Eigen::Matrix4f &p )
{
	globalPose = p;
	if (selection && selection->getNumChildren()>0)
	{
		SoMatrixTransform *mTr = dynamic_cast<SoMatrixTransform*>(selection->getChild(0));
		if (mTr)
		{
			SbMatrix m(reinterpret_cast<SbMat*>(globalPose.data()));
			mTr->matrix.setValue(m);
		}
		
	}
}*/

} // namespace VirtualRobot
