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
* @date       2011-02-24
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*
*
*/

#ifndef _VirtualRobot_CDManager_h_
#define _VirtualRobot_CDManager_h_

#include "../VirtualRobot.h"
#include "CollisionModel.h"
#include "../SceneObjectSet.h"
#include "CollisionChecker.h"

#include <vector>
#include <set>
#include <string>


namespace VirtualRobot {
/*!
*
* A framework that can handle different sets of collision models.
* With a collision detection manager (cdm) multiple sets collision models can be specified for convenient collision
* detection or distance calculations. All sets are checked against each other.
*
* @see CollsionModelSet 
*/
class VIRTUAL_ROBOT_IMPORT_EXPORT CDManager
{
public:
	//! if pColChecker is not set, the global collision checker is used
	CDManager(CollisionCheckerPtr colChecker = CollisionCheckerPtr());

	virtual ~CDManager();

	/*!
		This is the main method for adding collision models.
	*/
	void addCollisionModel (SceneObjectSetPtr m);

	/*! 
		Here single collision models can be added.
		Limitation: The models that are added here are all stored in one set and thus they are not checked against each other.
					Use separate SceneObjectSets as containers for the single collision models, 
					if they should be checked against each other.
	*/
	void addCollisionModel (SceneObjectPtr m);


	//! Returns true if one of the added ColModels collides with another colModel
	bool isInCollision();

	/*! 
		Checks if the model m collides with one of the added colModels.
		It is allowed to use an already added CollisionModel.
		Returns true if there is a collision.
	*/
	bool isInCollision (SceneObjectSetPtr m);
	
	//! Returns minimum distance of all colModels to each other.
	float getDistance();

	/*! 
		Calculates the shortest distance of m to the added colModels.
	    m may be an already added CollisionModel.
	*/
	float getDistance (SceneObjectSetPtr m);

	//! Stores min dist position and collision IDs
	float getDistance(Eigen::Vector3f &P1, Eigen::Vector3f &P2, int &trID1, int &trID2);

	/*! 
		Calculates the shortest distance of CollisionModel m to environment and to the added colModels.
		Stores nearest positions and corresponding IDs, where P1 and trID1 is used to store the data of m and 
		P2 and trID2 is used to store the data of this ccm.
	*/
	float getDistance(SceneObjectSetPtr m, Eigen::Vector3f &P1, Eigen::Vector3f &P2, int &trID1, int &trID2);

	/*!
		In the CollisionMdoelSet that can be accessed with this method, all CollisionModels have been added without being part of a SceneObjectSet.
	*/
	SceneObjectSetPtr getSingleCollisionModels();
	
	//! All SceneObjectSets
	std::vector<SceneObjectSetPtr> getSceneObjectSets();
		
	CollisionCheckerPtr getCollisionChecker();

protected:

	std::vector< SceneObjectSetPtr > colModels;
	CollisionCheckerPtr colChecker;

	//! if CollisionModels (not hosted by a collection) are added, they are stored here
	SceneObjectSetPtr singleCollisionModels;

};

}

#endif // _VirtualRobot_CDManager_h_

