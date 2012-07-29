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
* @package    SimDynamics
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#ifndef _SimDynamics_BulletCoinQtViewer_h_
#define _SimDynamics_BulletCoinQtViewer_h_

#include "../../SimDynamics.h"
#include "../../DynamicsWorld.h"
#include "BulletEngine.h"

#include <btBulletDynamicsCommon.h>
#include <LinearMath/btQuickprof.h>

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSelection.h>

#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

namespace SimDynamics
{

class SIMDYNAMICS_IMPORT_EXPORT BulletCoinQtViewer
{
public:
	BulletCoinQtViewer(DynamicsWorldPtr world);
	virtual ~BulletCoinQtViewer();

	/*!
		Call this method to initialize the 3d viewer.
		\param embedViewer The 3d viewer will be embedded in this QFrame.
		\param scene The scene that should be displayed.
	*/
	virtual void initSceneGraph(QFrame* embedViewer, SoNode* scene);

	void viewAll();

	/*!
		Visualize the dynamics object.
	*/
	void addVisualization(DynamicsObjectPtr o, VirtualRobot::SceneObject::VisualizationType visuType = VirtualRobot::SceneObject::Full);
	void addVisualization(DynamicsRobotPtr r, VirtualRobot::SceneObject::VisualizationType visuType = VirtualRobot::SceneObject::Full);

	/*!
		Remove the visualization of the dynamics object.
	*/
	void removeVisualization(DynamicsObjectPtr o);
	void removeVisualization(DynamicsRobotPtr r);

	//! Stop callbacks
	void stopCB();

protected:

	
	/*!
		This method is called periodically, triggered by a timer callback.
		It can be overwritten in order to perform custom updates.
		It is safe to access the scene graph.
	*/
	virtual void customUpdate(){};
	/*!
		This method is called when a node has been selected by the user.
		It can be overwritten to implement custom reactions.
		It is safe to access the scene graph.
		\param path The path that was selected
	*/
	virtual void customSelection(SoPath *path)
	{
		std::cout << "Selecting node " <<  path->getTail()->getTypeId().getName().getString() << endl;
	};
	virtual void customDeselection(SoPath *path)
	{
		std::cout << "Deselecting node " <<  path->getTail()->getTypeId().getName().getString() << endl;
	};

	//! Redraw
	virtual void scheduleRedraw();

	btScalar getDeltaTimeMicroseconds();

	void updateMotors(float dt);
	virtual void stepPhysics();

	static void timerCB(void * data, SoSensor * sensor);
	static void selectionCB( void *userdata, SoPath *path );
	static void deselectionCB( void *userdata, SoPath *path );
	SoQtExaminerViewer *viewer;
	SoTimerSensor *timerSensor;

	BulletEnginePtr bulletEngine;
	btClock m_clock;

	std::map<DynamicsObjectPtr,SoNode*> addedVisualizations;
	std::map<DynamicsRobotPtr,SoNode*> addedRobotVisualizations;

	SoSeparator* sceneGraphRoot;
	SoSeparator* floor;
	SoSelection* sceneGraph;
};


typedef boost::shared_ptr<BulletCoinQtViewer> BulletCoinQtViewerPtr;

} // namespace

#endif // _SimDynamics_BulletCoinQtViewer_h_
