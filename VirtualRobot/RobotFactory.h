/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_RobotFactory_h_
#define _VirtualRobot_RobotFactory_h_

#include "VirtualRobotImportExport.h"
#include "MathTools.h"

#include <string>
#include <vector>
#include <map>

namespace VirtualRobot
{

class Robot;
class RobotNode;

class VIRTUAL_ROBOT_IMPORT_EXPORT RobotFactory
{
public:
	/*!
	Creates an empty robot.
	*/
	static RobotPtr createRobot(const std::string &name);

	/*!
		Initializes Robot and all RobotNodes. 
		\param robotNodes All nodes of the robot. Must contain rootNode.
		\param childrenMap Parent-child relations are built according to this data.
		\param rootNode The root.
	*/
	static bool initializeRobot(RobotPtr robot, 
								std::vector<RobotNodePtr > &robotNodes, 
								std::map< RobotNodePtr, std::vector<std::string> > childrenMap,
								RobotNodePtr rootNode);
protected:
	// instantiation not allowed
	RobotFactory();
	virtual ~RobotFactory();

	//static bool initRobotNode(RobotNodePtr n, RobotNodePtr parent, std::vector< RobotNodePtr > &robotNodes);
};

}

#endif // _VirtualRobot_RobotFactory_h_
