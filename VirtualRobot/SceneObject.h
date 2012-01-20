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
#ifndef _VirtualRobot_SceneObject_h_
#define _VirtualRobot_SceneObject_h_

#include "VirtualRobot.h"
#include "VirtualRobotImportExport.h"
#include "Visualization/VisualizationNode.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/mpl/assert.hpp>

namespace VirtualRobot
{
class Robot;

class VIRTUAL_ROBOT_IMPORT_EXPORT SceneObject
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum VisualizationType
	{
		Full,			//!< the full model
		Collision,		//!< the collision model
		CollisionData	//!< a visualization of the collision model data that is internally used (this mode is only for debug purposes, the model is static, i.e. updates/movements/rotations are not visualized!)
	};

	struct Physics
	{
		enum CoMLocation
		{
			eCustom,			//!< Not related to model
			eVisuBBoxCenter		//!< The CoM position is computed from the bounding box of the visualization model
		};
		Physics()
		{

			localCoM.setZero();
			massKg = 0.0f;
			comLocation = eCustom;
		}
		void print()
		{
			std::cout << " ** Mass: " << massKg << " kg" << std::endl;
			std::cout << " ** local CoM: " <<  localCoM(0) << localCoM(1) << localCoM(2) << std::endl;
		}
		Eigen::Vector3f localCoM;	//!< Defined in the local coordinate system of this object
		float massKg;				//!< The mass of this object
		CoMLocation comLocation;	//!< Where is the CoM located
	};

	/*!
	*/
	SceneObject(const std::string &name, VisualizationNodePtr visualization = VisualizationNodePtr(), CollisionModelPtr collisionModel = CollisionModelPtr(), const Physics &p = Physics(), CollisionCheckerPtr colChecker = CollisionCheckerPtr());

	/*!
	*/
	virtual ~SceneObject();


	std::string getName() const;

	/*!
		Rename this object
	*/
	void setName(const std::string &name);

	/*!
		The global pose defines the position of the joint in the world. This value is used for visualization.
	*/
	virtual Eigen::Matrix4f getGlobalPose() const;
	virtual void setGlobalPose(const Eigen::Matrix4f &pose);

	virtual CollisionModelPtr getCollisionModel();
	virtual CollisionCheckerPtr getCollisionChecker();

	/*!
		Sets the main visualization of this object. 
	*/
	void setVisualization(VisualizationNodePtr visualization);
	void setCollisionModel(CollisionModelPtr colModel);

	/*!
		Return visualization object.
		\param visuType Set the type of visualization.
	*/
	virtual VisualizationNodePtr getVisualization(SceneObject::VisualizationType visuType = SceneObject::Full);

	virtual bool initialize();
	virtual void reset();

	/*!
		Enables/Disables the visualization updates of collision model and visualization model.
	*/
	void setUpdateVisualization (bool enable);
	bool getUpdateVisualizationStatus();

	/*!
		Setup the visualization of this object.
		\param showVisualization If false, the visualization is disabled.
		\param showAttachedVisualizations If false, the visualization of any attached optional visualizations is disabled.
	*/
	virtual void setupVisualization(bool showVisualization, bool showAttachedVisualizations);

	
	/*!
		Display the coordinate system of this object.
		If the object does not own a visualization yet, the VisualizationFactory is queried to get the first registered
		VisualizationType in order to build a valid visualization.
		\p enable Show or hide coordinate system
		\p scaling Size of coordinate system
		\p text Text to display at coordinate system. If not given, the name of this robot node will be displayed.
	*/
	virtual void showCoordinateSystem( bool enable, float scaling = 1.0f, std::string *text = NULL);

	/*!
		Returns true when the coordinate system is currently shown.
	*/
	virtual bool showCoordinateSystemState();

		
	/*!
		Display the bounding box of this object's collisionModel. 
		The bbox is not updated when you move the object, so you have to call this method again after touching the scene in order to ensure a correct visualization. 
		If the object does not own a visualization yet, the VisualizationFactory is queried to get the first registered
		VisualizationType in order to build a valid visualization.
		\p enable Show or hide bounding box
	*/
	virtual void showBoundingBox( bool enable );



	/*! 
		Builds a dummy visualization if necessary.
		\param visualizationType	If given the visualization is forced to be this type. 
									If not set, the first registered visualization from VisualizationFactory is used.
	*/
	virtual bool ensureVisualization(const std::string &visualizationType = "");

    /*!
        Transforms the pose, given in global coordinate system, to the local coordinate system of this object.
        @param poseGlobal The pose, given in global coordinate system, that should be transformed to the local coordinate system of this joint.
        @return The transformed pose.
    */
    Eigen::Matrix4f toLocalCoordinateSystem(const Eigen::Matrix4f &poseGlobal) const;
	Eigen::Vector3f toLocalCoordinateSystem(const Eigen::Vector3f &positionGlobal) const;
    /*!
        Transforms the pose, given in local coordinate system, to the global coordinate system.
        @param poseLocal The pose, given in local coordinate system of this joint, that should be transformed to the global coordinate system.
        @return The transformed pose.
    */
    Eigen::Matrix4f toGlobalCoordinateSystem(const Eigen::Matrix4f &poseLocal) const;
	Eigen::Vector3f toGlobalCoordinateSystem(const Eigen::Vector3f &positionLocal) const;

 	/*!
		Returns the transformation matrix from this object to otherObject
	*/
    Eigen::Matrix4f getTransformationTo(const SceneObjectPtr otherObject);


 	/*!
		Returns the transformation matrix from otherObject to this object
	*/
	Eigen::Matrix4f getTransformationFrom(const SceneObjectPtr otherObject);

	/*!
		Transform pose to local coordinate system of this object
	*/
    Eigen::Matrix4f transformTo(const SceneObjectPtr otherObject, const Eigen::Matrix4f &poseInOtherCoordSystem);

	/*!
		Transform position to local coordinate system of this object
	*/
    Eigen::Vector3f transformTo(const SceneObjectPtr otherObject, const Eigen::Vector3f &positionInOtherCoordSystem);

	/*! 
		get number of faces (i.e. triangles) of this object
		\p collisionModel Indicates weather the faces of the collision model or the full model should be returned.
	*/
	virtual int getNumFaces(bool collisionModel = false);

	/*!
		Return Center of Mass.
	*/
	Eigen::Vector3f getCoMLocal();
	Eigen::Vector3f getCoMGlobal();

	/*!
		Mass in Kg
	*/
	float getMass();

	virtual void print(bool printDecoration = true);


	/*!
		Retrieve a visualization in the given format.
		Example usage:
		 boost::shared_ptr<VirtualRobot::CoinVisualization> visualization = robot->getVisualization<CoinVisualization>();
		 SoNode* visualisationNode = NULL;
		 if (visualization)
		     visualisationNode = visualization->getCoinVisualization();

		@see CoinVisualization::getCoinVisualization() for convenient access!
	*/
	template <typename T> boost::shared_ptr<T> getVisualization(SceneObject::VisualizationType visuType = SceneObject::Full);

	/*!
	  Convenient method for highlighting the visualization of this object.
	  It is automatically checked whether the collision model or the full model is part of the visualization.
	  \param visualization The visualization for which the highlighting should be performed.
	  \param enable Set or unset highlighting.
	*/
	void highlight (VisualizationPtr visualization, bool enable);

	/*!
		Clones this object. If no col checker is given, the one of the original object is used.
	*/
	SceneObjectPtr clone( const std::string &name, CollisionCheckerPtr colChecker );

protected:

	SceneObject(){};
	///////////////////////// SETUP ////////////////////////////////////
	std::string name;
	bool initialized;														//< Invalid object when false
	///////////////////////// SETUP ////////////////////////////////////

	Eigen::Matrix4f globalPose;												//< The transformation that is used for visualization

	CollisionModelPtr collisionModel;
	VisualizationNodePtr visualizationModel;								//< This is the main visualization

	bool updateVisualization;

	virtual bool initializePhysics();
	Physics physics;

	CollisionCheckerPtr collisionChecker;
};


/**
 * This method creates a new Visualization
 * subclass which is given by the template parameter T.
 * T must be a subclass of VirtualRobot::Visualization.
 * A compile time error is thrown if a different class type is used as template argument.
 */
template <typename T>
boost::shared_ptr<T> SceneObject::getVisualization(SceneObject::VisualizationType visuType)
{
	const bool IS_SUBCLASS_OF_VISUALIZATION = ::boost::is_base_of<Visualization, T>::value;
	BOOST_MPL_ASSERT_MSG(IS_SUBCLASS_OF_VISUALIZATION, TEMPLATE_PARAMETER_FOR_VirtualRobot_getVisualization_MUST_BT_A_SUBCLASS_OF_VirtualRobot__Visualization, (T));
	boost::shared_ptr<T> visualization(new T(getVisualization(visuType)));
	return visualization;
}

} // namespace VirtualRobot

#endif // _VirtualRobot_SceneObject_h_
