/**
* @package    VirtualRobot
* @author     Manfred Kroehnert <mkroehnert _at_ users dot sourceforge dot net>
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Manfred Kroehnert
*/


#include "VisualizationNode.h"
#include "TriMeshModel.h"

#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/VirtualRobotException.h"

namespace VirtualRobot {

VisualizationNode::VisualizationNode()
{
		updateVisualization = true;
		showVisualization = true;
		showAttachedVisualizations = true;
}
	
VisualizationNode::~VisualizationNode()
{
}

VirtualRobot::VisualizationNodePtr VisualizationNode::clone(bool deepCopy)
{
	VisualizationNodePtr p(new VisualizationNode());
	p->setUpdateVisualization(updateVisualization);
	p->setFilename(filename,boundingBox);

	return p;
}

VirtualRobot::TriMeshModelPtr VisualizationNode::getTriMeshModel()
{
	return TriMeshModelPtr();
}

void VisualizationNode::attachVisualization(const std::string &name, VisualizationNodePtr v)
{
	THROW_VR_EXCEPTION_IF(!v,"NULL DATA");

	attachedVisualizations[name] = v;
}

void VisualizationNode::detachVisualization(const std::string &name)
{
	attachedVisualizations.erase(name);
}

bool VisualizationNode::hasAttachedVisualization(const std::string &name)
{
	std::map< std::string, VisualizationNodePtr >::const_iterator i = attachedVisualizations.begin();
	while (i!=attachedVisualizations.end())
	{
		if (i->first == name)
		{
			return true;
		}
		i++;
	}
	return false;
}

VisualizationNodePtr VisualizationNode::getAttachedVisualization(const std::string &name)
{
	if (!hasAttachedVisualization(name))
		return VisualizationNodePtr();
	return attachedVisualizations[name];
}


void VisualizationNode::setUpdateVisualization( bool enable )
{
	updateVisualization = enable;
}

bool VisualizationNode::getUpdateVisualizationStatus()
{
	return updateVisualization;
}

void VisualizationNode::print()
{
	cout << "Dummy VisualizationNode" << endl;
}

void VisualizationNode::setupVisualization( bool showVisualization, bool showAttachedVisualizations )
{
	this->showVisualization = showVisualization;
	this->showAttachedVisualizations = showAttachedVisualizations;
}

int VisualizationNode::getNumFaces()
{
	TriMeshModelPtr p = getTriMeshModel();
	if (p)
		return p->faces.size();
	return 0;
}

void VisualizationNode::setFilename(const std::string &filename, bool boundingBox)
{
	this->filename = filename;
	this->boundingBox = boundingBox;
}

bool VisualizationNode::usedBoundingBoxVisu()
{
	return boundingBox;
}

std::string VisualizationNode::getFilename()
{
	return filename;
}

} // namespace VirtualRobot
