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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010, 2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_CoinVisualizationFactory_h_
#define _VirtualRobot_CoinVisualizationFactory_h_


#include "../../VirtualRobotImportExport.h"
#include "../VisualizationFactory.h"
#include "../../BoundingBox.h"
#include "../../SceneObject.h"
#include "../../EndEffector/EndEffector.h"
#include "../ColorMap.h"

#include <Inventor/SoInput.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/SoOffscreenRenderer.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoCamera.h>

#include <boost/shared_ptr.hpp>

#include <string>

namespace VirtualRobot
{
class VisualizationNode;

class VIRTUAL_ROBOT_IMPORT_EXPORT CoinVisualizationFactory  : public VisualizationFactory
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CoinVisualizationFactory();
	virtual ~CoinVisualizationFactory();

	virtual VisualizationNodePtr getVisualizationFromFile(const std::string& filename, bool boundingBox = false);
	virtual VisualizationNodePtr getVisualizationFromString(const std::string& modelString, bool boundingBox = false);
	virtual VisualizationNodePtr createBox(float width, float height, float depth, float colorR, float colorG, float colorB, CollisionCheckerPtr colChecker = CollisionCheckerPtr());
	virtual VisualizationNodePtr createLine(const Eigen::Vector3f &from, const Eigen::Vector3f &to, float width = 1.0f, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
	virtual VisualizationNodePtr createLine(const Eigen::Matrix4f &from, const Eigen::Matrix4f &to, float width = 1.0f, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
	virtual VisualizationNodePtr createSphere(float radius, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f, CollisionCheckerPtr colChecker = CollisionCheckerPtr());
	virtual VisualizationNodePtr createCoordSystem(float scaling = 1.0f, std::string *text = NULL, float axisLength = 100.0f, float axisSize = 3.0f, int nrOfBlocks = 10);
	virtual VisualizationNodePtr createBoundingBox(const BoundingBox &bbox, bool wireFrame=false);
	virtual VisualizationNodePtr createVertexVisualization(const Eigen::Vector3f &position, float radius, float transparency,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
	virtual VisualizationNodePtr createTriMeshModelVisualization(TriMeshModelPtr model, bool showNormals, Eigen::Matrix4f &pose);
	virtual VisualizationNodePtr createPlane(const Eigen::Vector3f &position, const Eigen::Vector3f &normal, float extend, float transparency,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
	virtual VisualizationNodePtr createArrow(const Eigen::Vector3f &n, float length = 50.0f, float width = 2.0f, const Color &color = Color::Gray());

	/*!
		Create an empty VisualizationNode.
	*/
	virtual VisualizationNodePtr createVisualization(CollisionCheckerPtr colChecker = CollisionCheckerPtr());


	static SoSeparator* CreateConvexHull2DVisualization(const MathTools::ConvexHull2DPtr ch, MathTools::Plane &p, VisualizationFactory::Color colorInner = VisualizationFactory::Color::Blue(), VisualizationFactory::Color colorLine = VisualizationFactory::Color::Black(), float lineSize = 5.0f, const Eigen::Vector3f &offset =Eigen::Vector3f::Zero() );
	static SoSeparator* CreatePolygonVisualization(const std::vector<Eigen::Vector3f> &points, VisualizationFactory::Color colorInner = VisualizationFactory::Color::Blue(), VisualizationFactory::Color colorLine = VisualizationFactory::Color::Black(), float lineSize = 5.0f);
	static SoSeparator* CreatePlaneVisualization(const Eigen::Vector3f &position, const Eigen::Vector3f &normal, float extend, float transparency, bool grid=true,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
	static SoSeparator* CreateCoordSystemVisualization(float scaling = 1.0f, std::string *text = NULL, float axisLength = 100.0f, float axisSize = 3.0f, int nrOfBlocks = 10);
	static SoSeparator* CreateBoundingBox(SoNode* ivModel, bool wireFrame=false);
	static SoSeparator* CreateGrid(float width,float depth,float widthMosaic,float depthMosaic,bool InvertNormal,const char* pFileName,float Transparency);
	static SoSeparator* CreateBBoxVisualization(const BoundingBox &bbox, bool wireFrame=false);
	static SoSeparator* CreatePointVisualization(const MathTools::ContactPoint &point, bool showNormals = false);
	static SoSeparator* CreatePointsVisualization(const std::vector<MathTools::ContactPoint> &points, bool showNormals = false);
	static SoSeparator* CreateArrow(const Eigen::Vector3f &n, float length = 50.0f, float width = 2.0f, const Color &color = Color::Gray());
	static SoSeparator* CreateVertexVisualization( const Eigen::Vector3f &position, float radius, float transparency, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f );
	/*!
		Creates a colored model, by creating a new SoSeparator and adding a basecolor with overide flags followed by the model.
	*/
	static SoSeparator* Colorize( SoNode *model, VisualizationFactory::Color c);

	static SbMatrix getSbMatrix(Eigen::Matrix4f &m);
	static SbMatrix getSbMatrix(const Eigen::Vector3f &m);

	/*!
		Create a visualization of a set of grasps.
		\param graspSet The grasps to visualize
		\param eef The visualization of this eef is used to visualize the grasps
		\param pose The grasp set is visualized relatively to this pose (e.g. use the object position here)
		\param visu The visualization type of the EEFs.
	*/
	static SoSeparator* CreateGraspSetVisualization(GraspSetPtr graspSet, EndEffectorPtr eef, const Eigen::Matrix4f &pose = Eigen::Matrix4f::Identity(), SceneObject::VisualizationType visu = SceneObject::Full);
	static SoSeparator* CreateGraspVisualization( GraspPtr grasp, SoSeparator *eefVisu, const Eigen::Matrix4f &pose = Eigen::Matrix4f::Identity() );
	static SoSeparator* CreateGraspVisualization( GraspPtr grasp, EndEffectorPtr eef, const Eigen::Matrix4f &pose = Eigen::Matrix4f::Identity(), SceneObject::VisualizationType visu = SceneObject::Full );

	/*!
		Create a visualization of the end effector. 
		The visualization is moved, so that the origin is identical with the coordinate system of the TCP.
	*/
	static SoSeparator* CreateEndEffectorVisualization(EndEffectorPtr eef, SceneObject::VisualizationType = SceneObject::Full);

	/*!
		Convenient method to retrieve a coin visualization for a robot
	*/
	static SoNode *getCoinVisualization(RobotPtr robot, SceneObject::VisualizationType visuType);
	/*!
		Convenient method to retrieve a coin visualization for a SceneObject/Obstacle/ManipulationObject
	*/
	static SoNode *getCoinVisualization(SceneObjectPtr object, SceneObject::VisualizationType visuType);

	/*!
		Convenient method to retrieve a coin visualization for a set of contacts
	*/
	static SoNode *getCoinVisualization(std::vector <EndEffector::ContactInfo> &contacts);
	static SoNode *getCoinVisualization(EndEffector::ContactInfo &contact);
	
	static SoNode *getCoinVisualization(VisualizationNodePtr visu);

	static SoNode *getCoinVisualization(TriMeshModelPtr model, bool shownormals, VisualizationFactory::Color color = VisualizationFactory::Color::Gray());

	/*!
		Create a visualization of the reachability space.
	*/
	static SoNode* getCoinVisualization(ReachabilitySpacePtr reachSpace, const VirtualRobot::ColorMap::type cmType, bool transformToGlobalPose = true);
	/*! 
		Creates a visualization of the reachability space. For each 3D point, the orientation with maximum entry is determined and visualized as an arrow. The direction of this arrow is aligned to the param axis.
	*/
	static SoNode* getCoinVisualization(ReachabilitySpacePtr reachSpace, VirtualRobot::ColorMap::type cmType, const Eigen::Vector3f &axis, bool transformToGlobalPose = true, unsigned char minValue = 0, float arrowSize = 0);	
	//! Helper method: Create reach space visualization of a fixed orientation
	static SoNode* getCoinVisualization(ReachabilitySpacePtr reachSpace, const Eigen::Vector3f &fixedEEFOrientationGlobalRPY, VirtualRobot::ColorMap::type cmType = VirtualRobot::ColorMap::eHot, bool transformToGlobalPose = true, const Eigen::Vector3f &axis = Eigen::Vector3f(0,0,1.0f), unsigned char minValue = 0, float arrowSize = 0);
	//! helper method: create nrBestEntries arrows in direction of maximum orientation for voxelPosition (a,b,c)
	static SoNode* getCoinVisualization(ReachabilitySpacePtr reachSpace, int a, int b, int c, int nrBestEntries, SoSeparator* arrow, const VirtualRobot::ColorMap &cm, bool transformToGlobalPose, unsigned char minValue);

	static SoMatrixTransform* getMatrixTransform(Eigen::Matrix4f &m);
	static SoNode* createCoinLine(const Eigen::Matrix4f &from, const Eigen::Matrix4f &to, float width, float colorR, float colorG, float colorB);

	static SoOffscreenRenderer* createOffscreenRenderer(int width, int height);
	static bool renderOffscreen( SoOffscreenRenderer* renderer, SoCamera* cam, SoNode* scene, unsigned char **buffer);
	static bool renderOffscreen( SoOffscreenRenderer* renderer, RobotNodePtr camNode, SoNode* scene, unsigned char **buffer );
protected:
	static void GetVisualizationFromSoInput(SoInput& soInput, VisualizationNodePtr& visualizationNode, bool bbox = false);

	static inline char IVToolsHelper_ReplaceSpaceWithUnderscore(char input) { if ( ' ' == input ) return '_'; else return input; }

// AbstractFactoryMethod
public:
	static std::string getName();
	static boost::shared_ptr<VisualizationFactory> createInstance(void*);
private:
	static SubClassRegistry registry;
};

} // namespace VirtualRobot

#endif // _VirtualRobot_CoinVisualizationFactory_h_
