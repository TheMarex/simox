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
#ifndef _VirtualRobot_Obstacle_h_
#define _VirtualRobot_Obstacle_h_

#include "VirtualRobotImportExport.h"

#include <string>
#include <vector>

#include "CollisionDetection/CollisionModel.h"
#include "SceneObject.h"

namespace VirtualRobot 
{

class VIRTUAL_ROBOT_IMPORT_EXPORT Obstacle : public SceneObject
{
public:

	/*!
	*/
	Obstacle(const std::string &name, VisualizationNodePtr visualization = VisualizationNodePtr(), CollisionModelPtr collisionModel = CollisionModelPtr(), const SceneObject::Physics &p = SceneObject::Physics(), CollisionCheckerPtr colChecker = CollisionCheckerPtr());

	/*!
	*/
	virtual ~Obstacle();

	virtual void print(bool printDecoration = true);

	/*!
		Clones this object. If no col checker is given, the one of the original object is used.
	*/
	ObstaclePtr clone( const std::string &name, CollisionCheckerPtr colChecker = CollisionCheckerPtr() );

	int getID();

	/*!
		Create a standard obstacle. 

	*/
	static ObstaclePtr createBox(float width, float height, float depth, float colorR = 1.0f, float colorG = 0.0f, float colorB=0.0f, std::string visualizationType = "inventor", CollisionCheckerPtr colChecker = CollisionCheckerPtr());

protected:

	// a counter for internal ids
	static int idCounter;
	// my id
	int id;
};

} // namespace

#endif // _VirtualRobot_Obstacle_h_
