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
#ifndef _VirtualRobot_Robot_h_
#define _VirtualRobot_Robot_h_

#include "VirtualRobotImportExport.h"
#include "Nodes/RobotNode.h"
#include "RobotNodeSet.h"

#include <boost/enable_shared_from_this.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/mpl/assert.hpp>
#include <boost/bind.hpp>

#include <string>
#include <map>
#include <vector>

#include <boost/mem_fn.hpp>
#include <algorithm>

#include <Eigen/Core>

namespace VirtualRobot
{
class Visualization;

class VIRTUAL_ROBOT_IMPORT_EXPORT Robot : public boost::enable_shared_from_this<Robot>
{
	friend class RobotIO;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
	Constructor
	\param name Specifies the name of this instance.
	\param type Specifies the type of the robot (e.g. multiple robots of the same type could exists with different names)
	*/
	Robot(const std::string &name, const std::string &type="");

	/*!
	*/
	virtual ~Robot();

	/*!
		The root node is the first RobotNode of this robot.
	*/
	void setRootNode(RobotNodePtr node);
	RobotNodePtr getRootNode();

	void applyJointValues();

	std::vector< RobotNodePtr > getRobotNodes();
	void getRobotNodes(std::vector< RobotNodePtr > &storeNodes, bool clearVector=true);


	/*!
		Retrieve a visualization in the given format.
		Example usage:
		 boost::shared_ptr<VirtualRobot::CoinVisualization> visualization = robot->getVisualization<CoinVisualization>();
		 SoNode* visualisationNode = NULL;
		 if (visualization)
		     visualisationNode = visualization->getCoinVisualization();
	*/
	template <typename T> boost::shared_ptr<T> getVisualization(SceneObject::VisualizationType visuType = SceneObject::Full);
	/*! 
		Shows the structure of the robot
	*/
	void showStructure(bool enable, const std::string &type="inventor");
	/*! 
		Shows the coordinate systems of the robot nodes
	*/
	void showCoordinateSystems(bool enable, const std::string &type="inventor");

	/*!
		Setup the full model visualization.
		\param showVisualization If false, the visualization is disabled.
		\param showAttachedVisualizations If false, the visualization of any attached optional visualizations is disabled.
	*/
	void setupVisualization(bool showVisualization, bool showAttachedVisualizations);


	std::string getName();
	std::string getType();

	/*!
	Print status information.
	*/
	virtual void print();

	/*!
		Enables/Disables the visualization updates of collision model and visualization model.
	*/
	void setUpdateVisualization (bool enable);
	bool getUpdateVisualizationStatus();


	/*!
		This method is automatically called in RobotNode's initialization routine.
	*/
	void registerRobotNode(RobotNodePtr node);
	void deregisterRobotNode(RobotNodePtr node);
	bool hasRobotNode( RobotNodePtr node );
	bool hasRobotNode( const std::string &robotNodeName );
	RobotNodePtr getRobotNode(const std::string &robotNodeName);

	/*!
		This method is automatically called in RobotNodeSet's initialization routine.
	*/
	void registerRobotNodeSet(RobotNodeSetPtr nodeSet);
	void deregisterRobotNodeSet(RobotNodeSetPtr nodeSet);
	bool hasRobotNodeSet( RobotNodeSetPtr nodeSet );
	bool hasRobotNodeSet( const std::string &name );
	RobotNodeSetPtr getRobotNodeSet(const std::string &nodeSetName);
	std::vector<RobotNodeSetPtr> getRobotNodeSets();
	void getRobotNodeSets(std::vector<RobotNodeSetPtr> &storeNodeSet);

	/**
	 *
	 */
	void registerEndEffector(EndEffectorPtr endEffector);
	bool hasEndEffector(EndEffectorPtr endEffector);
	bool hasEndEffector(const std::string& endEffectorName);
	EndEffectorPtr getEndEffector(const std::string& endEffectorName);
	std::vector<EndEffectorPtr> getEndEffectors();
	void getEndEffectors(std::vector<EndEffectorPtr> &storeEEF);

	/*!
		Use this method to automatically build a SceneObjectSet out of a RobotNodeSet (e.g. to be used for collision detection)
		This method will create a new SceneObjectSet!
	*/
	SceneObjectSetPtr getSceneObjectSet(const std::string &robotNodeSet);
	std::vector< CollisionModelPtr > getCollisionModels();
		
	CollisionCheckerPtr getCollisionChecker();

	/*!
	  Convenient method for highlighting the visualization of this robot.
	  It is automatically checked whether the collision model or the full model is part of the visualization.
	  \param visualization The visualization for which the highlighting should be performed.
	  \param enable On or off
	*/
	virtual void highlight (VisualizationPtr visualization, bool enable);

	
	/*! 
		get number of faces (i.e. triangles) of this object
		\p collisionModel Indicates weather the faces of the collision model or the full model should be returned.
	*/
	virtual int getNumFaces(bool collisionModel = false);

	/*!
		Set the global position of this robot
	*/
	void setGlobalPose(const Eigen::Matrix4f &globalPose);

	/*!
		Set the global pose of this robot so that the RobotNode node is at position globalPoseNode
	*/
	void setGlobalPoseForRobotNode(const RobotNodePtr &node, const Eigen::Matrix4f &globalPoseNode);

	Eigen::Matrix4f getGlobalPose();

	/*!
		Return center of mass of this robot
	*/
	Eigen::Vector3f getCoM();

	/*!
		Return accumulated mass of this robot.
	*/
	float getMass();

	/*!
		Extract a sub kinematic from this robot and create a new robot instance.
		\param startJoint The kinematic starts with this RobotNode
		\param newRobotType The name of the newly created robot type
		\param newRobotName The name of the newly created robot
		\param cloneRNS Clone all robot node sets that belong to the original robot and for which the remaining robot nodes of the subPart are sufficient.
		\param cloneEEFs Clone all end effectors that belong to the original robot and for which the remaining robot nodes of the subPart are sufficient.
		\param collisionChecker The new robot can be registered to a different collision checker. If not set, the collision checker of the original robot is used.
	*/
	RobotPtr extractSubPart(RobotNodePtr startJoint, const std::string &newRobotType, const std::string &newRobotName, bool cloneRNS = true, bool cloneEEFs = true, CollisionCheckerPtr collisionChecker=CollisionCheckerPtr());

	/*!
		Clones this robot.
		\param name The new name.
		\param collisionChecker If set, the returned robot is registered with this col checker, otherwise the CollisionChecker of the original robot is used.

	*/
	RobotPtr clone(const std::string &name, CollisionCheckerPtr collisionChecker = CollisionCheckerPtr());

protected:
	/*!
		Goes through all RobotNodes and if no visualization is present:
		* the collision model is checked and in case it owns a visualization 
		* it is cloned and used as visualization.
	*/
	void createVisualizationFromCollisionModels();


	std::string name;
	std::string type;
	RobotNodePtr rootNode;

	bool updateVisualization;

	Eigen::Matrix4f globalPose; //!< The pose of this roobt in the world

	std::map< std::string, RobotNodePtr > robotNodeMap;
	std::map< std::string, RobotNodeSetPtr > robotNodeSetMap;
	std::map< std::string, EndEffectorPtr > endEffectorMap;
};

/**
 * This method collects all visualization nodes and creates a new Visualization
 * subclass which is given by the template parameter T.
 * T must be a subclass of VirtualRobot::Visualization.
 * A compile time error is thrown if a different class type is used as template argument.
 */
template <typename T>
boost::shared_ptr<T> Robot::getVisualization(SceneObject::VisualizationType visuType)
{
	const bool IS_SUBCLASS_OF_VISUALIZATION = ::boost::is_base_of<Visualization, T>::value;
	BOOST_MPL_ASSERT_MSG(IS_SUBCLASS_OF_VISUALIZATION, TEMPLATE_PARAMETER_FOR_VirtualRobot_getVisualization_MUST_BT_A_SUBCLASS_OF_VirtualRobot__Visualization, (T));
	std::vector<RobotNodePtr> collectedRobotNodes;
	getRobotNodes(collectedRobotNodes);
	std::vector<VisualizationNodePtr> collectedVisualizationNodes(collectedRobotNodes.size());
	for (size_t i=0;i<collectedRobotNodes.size();i++)
		collectedVisualizationNodes[i] = collectedRobotNodes[i]->getVisualization(visuType);

	boost::shared_ptr<T> visualization(new T(collectedVisualizationNodes));
	return visualization;
}

} // namespace VirtualRobot

#endif // _Robot_h_
