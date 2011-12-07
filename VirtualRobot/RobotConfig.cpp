
#include "RobotConfig.h"
#include "Robot.h"
#include "VirtualRobotException.h"


namespace VirtualRobot
{

RobotConfig::RobotConfig(RobotPtr robot, const std::string &name)
	: name(name),
	robot(robot)
{
	THROW_VR_EXCEPTION_IF(!robot,"NULL robot in RobotConfig");
}

RobotConfig::RobotConfig(RobotPtr robot, const std::string &name, const std::vector< Configuration > &configs)
	: name(name),
	robot(robot)
{
	THROW_VR_EXCEPTION_IF(!robot,"NULL robot in RobotConfig");
	for (std::vector< Configuration >::const_iterator i=configs.begin(); i!=configs.end(); i++ )
	{
		setConfig((*i));
	}
}

RobotConfig::RobotConfig(RobotPtr robot, const std::string &name, const std::map< RobotNodePtr, float > &configs)
	: name(name),
	robot(robot)
{
	THROW_VR_EXCEPTION_IF(!robot,"NULL robot in RobotConfig");
	for (std::map< RobotNodePtr, float >::const_iterator i=configs.begin(); i!=configs.end(); i++ )
	{
		setConfig(i->first,i->second);
	}
}

RobotConfig::RobotConfig(RobotPtr robot, const std::string &name, const std::vector< std::string > &robotNodes, const std::vector< float > &values)
	: name(name),
	robot(robot)
{
	THROW_VR_EXCEPTION_IF(!robot,"NULL robot in RobotConfig");
	THROW_VR_EXCEPTION_IF(robotNodes.size() != values.size(),"Vector sizes have to be equal in RobotConfig");
	for (size_t i=0;i<robotNodes.size();i++)
	{
		setConfig(robotNodes[i],values[i]);
	}
}

RobotConfig::RobotConfig(RobotPtr robot, const std::string &name, const std::vector< RobotNodePtr > &robotNodes, const std::vector< float > &values)
	: name(name),
	robot(robot)
{
	THROW_VR_EXCEPTION_IF(!robot,"NULL robot in RobotConfig");
	THROW_VR_EXCEPTION_IF(robotNodes.size() != values.size(),"Vector sizes have to be equal in RobotConfig");
	for (size_t i=0;i<robotNodes.size();i++)
	{
		setConfig(robotNodes[i],values[i]);
	}
}


void RobotConfig::print() const
{
	cout << "  Robot Config <" << name << ">" << endl;

	for (std::map< RobotNodePtr, float >::const_iterator i=configs.begin(); i!=configs.end(); i++ )
	{
		cout << "  * " << i->first->getName() << ":\t" << i->second << endl;
	}
}

void RobotConfig::setConfig( const Configuration &c )
{
	THROW_VR_EXCEPTION_IF(!robot,"Null data");
	setConfig(c.name,c.value);
}

void RobotConfig::setConfig( const std::string &node, float value )
{
	THROW_VR_EXCEPTION_IF(!robot,"Null data");
	RobotNodePtr rn = robot->getRobotNode(node);
	THROW_VR_EXCEPTION_IF(!rn,"Did not find robot node with name " << node);
	configs[rn] = value;
}

void RobotConfig::setConfig( RobotNodePtr node, float value )
{
	THROW_VR_EXCEPTION_IF(!robot,"Null data");
	THROW_VR_EXCEPTION_IF(!node,"Null data");
	THROW_VR_EXCEPTION_IF(!robot->hasRobotNode(node),"Robot node with name " << node->getName() << " does not belong to robot " << robot->getName());
	configs[node] = value;
}

VirtualRobot::RobotPtr RobotConfig::getRobot()
{
	return robot;
}

std::string RobotConfig::getName() const
{
	return name;
}

RobotConfigPtr RobotConfig::clone( RobotPtr newRobot )
{
	if (!newRobot)
		newRobot = robot;
	std::map< RobotNodePtr, float > newConfigs;
	std::map< RobotNodePtr, float >::iterator i = configs.begin();
	while (i!=configs.end())
	{
		RobotNodePtr rn = newRobot->getRobotNode(i->first->getName());
		if (!rn)
		{
			VR_WARNING << "Could not completely clone RobotConfig " << name << " because new robot does not know a RobtoNode with name " << i->first->getName() << endl;
		} else
		{
			newConfigs[rn] = i->second;
		}
		i++;
	}
	RobotConfigPtr result(new RobotConfig(newRobot,name,newConfigs));
	return result;
}


void RobotConfig::setJointValues()
{
	THROW_VR_EXCEPTION_IF(!robot,"Null data");
	for (std::map< RobotNodePtr, float >::const_iterator i=configs.begin(); i!=configs.end(); i++ )
	{
		i->first->setJointValue(i->second, false, true);
	}
	robot->applyJointValues();
}

bool RobotConfig::hasConfig( const std::string & name ) const
{
	for (std::map< RobotNodePtr, float >::const_iterator i=configs.begin(); i!=configs.end(); i++ )
	{
		if (i->first->getName() == name)
			return true;
	}

	return false;
}

float RobotConfig::getConfig( const std::string & name ) const
{
	if (!hasConfig(name))
		return 0.0f;
	RobotNodePtr rn = robot->getRobotNode(name);
	THROW_VR_EXCEPTION_IF(!rn,"Did not find robot node with name " << name);
	std::map< RobotNodePtr, float >::const_iterator i = configs.find(rn);
	if (i==configs.end())
	{
		VR_ERROR << "Internal error..." << endl;
		return 0.0f;
	}
	return i->second;
}

std::vector< RobotNodePtr > RobotConfig::getNodes() const
{
	std::vector< RobotNodePtr > result;
	std::map< RobotNodePtr, float >::const_iterator i = configs.begin();
	while (i != configs.end())
	{
		result.push_back(i->first);
		i++;
	}
	return result;
}

std::map < std::string, float > RobotConfig::getRobotNodeJointvalueMap()
{
	std::map < std::string, float > result;
	std::map< RobotNodePtr, float >::const_iterator i = configs.begin();
	while (i != configs.end())
	{
		result[i->first->getName()] = i->second;
		i++;
	}
	return result;
}

bool RobotConfig::applyToRobot( RobotPtr r )
{
	if (!r)
		return false;

	std::map < std::string, float > jv = getRobotNodeJointvalueMap();
	std::map< std::string, float >::const_iterator i = jv.begin();

	// first check if all nodes are present
	while (i != jv.end())
	{
		if (!r->hasRobotNode(i->first))
			return false;
		i++;
	}

	// apply jv
	i = jv.begin();
	while (i != jv.end())
	{
		RobotNodePtr rn = r->getRobotNode(i->first);
		if (!rn)
			return false;
		rn->setJointValue(i->second);
		i++;
	}
	robot->applyJointValues();
	return true;
}





} // namespace VirtualRobot
