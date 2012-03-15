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
#ifndef _VirtualRobot_Scene_h_
#define _VirtualRobot_Scene_h_

#include "VirtualRobotImportExport.h"
#include "SceneObject.h"
#include "Robot.h"
#include "RobotConfig.h"
#include "Nodes/RobotNode.h"
#include "Obstacle.h"
#include "ManipulationObject.h"
#include "RobotConfig.h"
#include <string>
#include <vector>
#include <Eigen/Core>

namespace VirtualRobot 
{

class VIRTUAL_ROBOT_IMPORT_EXPORT Scene
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
	*/
	Scene(const std::string &name);

	/*!
	*/
	virtual ~Scene();

	/*!
		Registers the robot to this scene. If a robot with the same name is already registered nothing happens. 
	*/
	void registerRobot(RobotPtr robot);

	/*!
		Removes the robot to from this scene. If the robot is not registered nothing happens. 
	*/
	void deRegisterRobot(RobotPtr robot);
	void deRegisterRobot(const std::string &name);

	bool hasRobot(RobotPtr robot) const;
	bool hasRobot(const std::string &name) const;

	RobotPtr getRobot(const std::string &name);

	std::vector< RobotPtr > getRobots();


	/*!
		Registers the RobotConfig to this scene. If a config  with the same name is already registered nothing happens. 
	*/
	void registerRobotConfig(RobotPtr robot, RobotConfigPtr config);
	void registerRobotConfig(RobotPtr robot, std::vector<RobotConfigPtr> configs);

	/*!
		Removes the RobotConfig to from this scene. If the RobotConfig is not registered nothing happens. 
	*/
	void deRegisterRobotConfig(RobotPtr robot, RobotConfigPtr config);
	void deRegisterRobotConfig(RobotPtr robot, const std::string &name);

	bool hasRobotConfig(RobotPtr robot, RobotConfigPtr config);
	bool hasRobotConfig(RobotPtr robot, const std::string &name);

	RobotConfigPtr getRobotConfig(const std::string &robotName, const std::string &name);
	RobotConfigPtr getRobotConfig(RobotPtr robot, const std::string &name);

	std::vector< RobotConfigPtr > getRobotConfigs(RobotPtr robot);

	/*!
		Registers the ManipulationObject to this scene. If an ManipulationObject with the same name is already registered nothing happens. 
	*/
	void registerManipulationObject(ManipulationObjectPtr obj);

	/*!
		Removes the ManipulationObject to from this scene. If the ManipulationObject is not registered nothing happens. 
	*/
	void deRegisterManipulationObject(ManipulationObjectPtr obj);
	void deRegisterManipulationObject(const std::string &name);

	bool hasManipulationObject(ManipulationObjectPtr obstacle) const;
	bool hasManipulationObject(const std::string &name) const;

	ManipulationObjectPtr getManipulationObject(const std::string &name);

	std::vector< ManipulationObjectPtr > getManipulationObjects();

	/*!
		Registers the obstacle to this scene. If an obstacle with the same name is already registered nothing happens. 
	*/
	void registerObstacle(ObstaclePtr obstacle);

	/*!
		Removes the obstacle to from this scene. If the obstacle is not registered nothing happens. 
	*/
	void deRegisterObstacle(ObstaclePtr obstacle);
	void deRegisterObstacle(const std::string &name);

	bool hasObstacle(ObstaclePtr obstacle) const;
	bool hasObstacle(const std::string &name) const;

	ObstaclePtr getObstacle(const std::string &name);

	std::vector< ObstaclePtr > getObstacles();



	
	/*!
		Registers the set to this scene. If a set with the same name is already registered nothing happens. 
	*/
	void registerSceneObjectSet(SceneObjectSetPtr sos);

	/*!
		Removes the set to from this scene. If the set is not registered nothing happens. 
	*/
	void deRegisterSceneObjectSet(SceneObjectSetPtr sos);
	void deRegisterSceneObjectSet(const std::string &name);

	bool hasSceneObjectSet(SceneObjectSetPtr sos) const;
	bool hasSceneObjectSet(const std::string &name) const;

	SceneObjectSetPtr getSceneObjectSet(const std::string &name);

	std::vector< SceneObjectSetPtr > getSceneObjectSets();





	RobotNodeSetPtr getRobotNodeSet(const std::string &robot, const std::string rns);

	std::string getName() const;

	/*!
		Retrieve a visualization in the given format.
		Example usage:
		 boost::shared_ptr<VirtualRobot::CoinVisualization> visualization = scene->getVisualization<CoinVisualization>();
		 SoNode* visualisationNode = NULL;
		 if (visualization)
		     visualisationNode = visualization->getCoinVisualization();
	*/
	template <typename T> boost::shared_ptr<T> getVisualization(SceneObject::VisualizationType visuType = SceneObject::Full);


	/*!
		Creates an XML string that describes this scene. 
		\param basePath All paths to robots or objects are stored relative to this path.
		\return The xml string.
	*/
	std::string getXMLString(const std::string &basePath);
protected:

	std::string name;

	std::vector< RobotPtr > robots;
	std::map< RobotPtr, std::vector< RobotConfigPtr > > robotConfigs;
	std::vector< ObstaclePtr > obstacles;
	std::vector< ManipulationObjectPtr > manipulationObjects;
	std::vector< SceneObjectSetPtr > sceneObjectSets;

};

/**
 * This method collects all visualization nodes and creates a new Visualization
 * subclass which is given by the template parameter T.
 * T must be a subclass of VirtualRobot::Visualization.
 * A compile time error is thrown if a different class type is used as template argument.
 */
template <typename T>
boost::shared_ptr<T> Scene::getVisualization(SceneObject::VisualizationType visuType)
{
	const bool IS_SUBCLASS_OF_VISUALIZATION = ::boost::is_base_of<Visualization, T>::value;
	BOOST_MPL_ASSERT_MSG(IS_SUBCLASS_OF_VISUALIZATION, TEMPLATE_PARAMETER_FOR_VirtualRobot_getVisualization_MUST_BT_A_SUBCLASS_OF_VirtualRobot__Visualization, (T));
	std::vector<VirtualRobot::RobotPtr> collectedRobots = getRobots();
	std::vector<VirtualRobot::ObstaclePtr> collectedObstacles = getObstacles();
	std::vector<VirtualRobot::ManipulationObjectPtr> collectedManipulationObjects = getManipulationObjects();

	// collect all robotnodes
	std::vector<VirtualRobot::RobotNodePtr> collectedRobotNodes;
	for (size_t i=0;i<collectedRobots.size();i++)
		collectedRobots[i]->getRobotNodes(collectedRobotNodes,false);

	std::vector<VisualizationNodePtr> collectedVisualizationNodes(collectedRobotNodes.size() + collectedObstacles.size() + collectedManipulationObjects.size());
	for (size_t i=0;i<collectedRobotNodes.size();i++)
		collectedVisualizationNodes[i] = collectedRobotNodes[i]->getVisualization(visuType);
	for (size_t i=0;i<collectedObstacles.size();i++)
		collectedVisualizationNodes[i+collectedRobotNodes.size()] = collectedObstacles[i]->getVisualization(visuType);
	for (size_t i=0;i<collectedManipulationObjects.size();i++)
		collectedVisualizationNodes[i+collectedRobotNodes.size()+collectedObstacles.size()] = collectedManipulationObjects[i]->getVisualization(visuType);
	

	boost::shared_ptr<T> visualization(new T(collectedVisualizationNodes));
	return visualization;
}

} // namespace

#endif // _VirtualRobot_Scene_h_
