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
#ifndef _VirtualRobot_ManipulationObject_h_
#define _VirtualRobot_ManipulationObject_h_

#include "VirtualRobotImportExport.h"

#include <string>
#include <vector>

#include "Obstacle.h"
#include "EndEffector/EndEffector.h"
#include <Eigen/Core>

namespace VirtualRobot 
{

class VIRTUAL_ROBOT_IMPORT_EXPORT ManipulationObject : public Obstacle
{
public:

	ManipulationObject(const std::string &name, VisualizationNodePtr visualization = VisualizationNodePtr(), CollisionModelPtr collisionModel = CollisionModelPtr(), const SceneObject::Physics &p = SceneObject::Physics(), CollisionCheckerPtr colChecker = CollisionCheckerPtr());

	/*!
	*/
	virtual ~ManipulationObject();

	virtual void print(bool printDecoration=true);

	bool hasGraspSet (GraspSetPtr graspSet);
	bool hasGraspSet (const std::string &robotType, const std::string &eef);

	/*!
		Appends a grasp set. Note, that only one grasp set per EEF is allowed.
	*/
	void addGraspSet(GraspSetPtr graspSet);

	/*!
		Get grasp set for the given end effector.
		An empty GraspSetPtr is returned when no GraspSet for eef is found.
	*/
	GraspSetPtr getGraspSet(EndEffectorPtr eef);

	/*!
		Get grasp set for the given robotType and end effector.
		An empty GraspSetPtr is returned when no GraspSet for robot&eef is found.
	*/
	GraspSetPtr getGraspSet(const std::string &robotType,const std::string &eefName);

	/*!
		Creates an XML representation of this object.
		\param basePath If set, all visualization and collision model files are made relative to this path.
		\param tabs Create indention at the beginning of each line.
		\param storeLinkToFile If set, the data (e.g. grasps) are not explicitly listed, but an xml tag directing to the XML file, 
				from which this instance was loaded, is set. If not set a deep description is created.
	*/
	std::string getXMLString(const std::string &basePath = std::string(), int tabs = 0, bool storeLinkToFile = false);

	/*!
		Clones this object. If no col checker is given, the one of the original object is used.
	*/
	ManipulationObjectPtr clone( const std::string &name, CollisionCheckerPtr colChecker = CollisionCheckerPtr() ) const {return ManipulationObjectPtr(_clone(name,colChecker));}

	void setFilename(const std::string &filename);
	std::string getFilename();

protected:

	virtual ManipulationObject* _clone( const std::string &name, CollisionCheckerPtr colChecker = CollisionCheckerPtr() ) const;


	std::string filename;
		
	std::vector< GraspSetPtr > graspSets;
};

} // namespace

#endif // _VirtualRobot_ManipulationObject_h_
