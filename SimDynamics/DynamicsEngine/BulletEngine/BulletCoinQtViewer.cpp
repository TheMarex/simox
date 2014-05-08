#include "BulletCoinQtViewer.h"

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include "Inventor/actions/SoBoxHighlightRenderAction.h"
#include <Inventor/nodes/SoSelection.h>

#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h>

#include <boost/foreach.hpp>

using namespace VirtualRobot;

namespace SimDynamics
{


BulletCoinQtViewer::BulletCoinQtViewer(DynamicsWorldPtr world)
{
	bulletTimeStepMsec = 16; // 60FPS
	bulletMaxSubSteps = 1;
	enablePhysicsUpdates = true;

	const float TIMER_MS = 30.0f;

	SIMDYNAMICS_ASSERT(world);

	bulletEngine = boost::dynamic_pointer_cast<BulletEngine>(world->getEngine());

	SIMDYNAMICS_ASSERT(bulletEngine);

	/*sceneGraph = new SoSeparator;*/
	sceneGraphRoot = new SoSeparator();
	sceneGraphRoot->ref();
	floor = new SoSeparator();
	sceneGraphRoot->addChild(floor);
	sceneGraph = new SoSelection();
	sceneGraphRoot->addChild(sceneGraph);

	//SoSelection *selection = new SoSelection();
	//sceneGraph->addChild( selection );
	viewer = NULL;

	// register callback
	SoSensorManager *sensor_mgr = SoDB::getSensorManager();
	timerSensor = new SoTimerSensor(timerCB, this);
	timerSensor->setInterval(SbTime(TIMER_MS/1000.0f));
	sensor_mgr->insertTimerSensor(timerSensor);

	// selection cb
	sceneGraph->addSelectionCallback( selectionCB, this );
	sceneGraph->addDeselectionCallback( deselectionCB, this );
}

BulletCoinQtViewer::~BulletCoinQtViewer()
{
	stopCB();
	sceneGraphRoot->unref();
	sceneGraphRoot = NULL;
}

void BulletCoinQtViewer::selectionCB( void *userdata, SoPath *path )
{
	BulletCoinQtViewer *bulletViewer = static_cast<BulletCoinQtViewer*>(userdata);
	VR_ASSERT(bulletViewer);

	VR_INFO << "Selected object" << endl;

	bulletViewer->customSelection(path);

	bulletViewer->scheduleRedraw();
}
void BulletCoinQtViewer::deselectionCB( void *userdata, SoPath *path )
{
	BulletCoinQtViewer *bulletViewer = static_cast<BulletCoinQtViewer*>(userdata);
	VR_ASSERT(bulletViewer);

	VR_INFO << "Deselected object" << endl;

	bulletViewer->customDeselection(path);

	bulletViewer->scheduleRedraw();
}

void BulletCoinQtViewer::timerCB(void * data, SoSensor * sensor)
{
	BulletCoinQtViewer *bulletViewer = static_cast<BulletCoinQtViewer*>(data);
	VR_ASSERT(bulletViewer);

	// now its safe to update physical information and set the models to the according poses
	bulletViewer->updatePhysics();

	// perform some custom updates if needed
	bulletViewer->customUpdate();

	bulletViewer->scheduleRedraw();

}

void BulletCoinQtViewer::initSceneGraph( QFrame* embedViewer, SoNode* scene )
{
	viewer = new SoQtExaminerViewer(embedViewer,"",TRUE,SoQtExaminerViewer::BUILD_POPUP);

	// setup
	viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
	viewer->setAccumulationBuffer(true);

	viewer->setAntialiasing(true, 4);

	viewer->setGLRenderAction(new SoBoxHighlightRenderAction);
	viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
	viewer->setFeedbackVisibility(true);
	if (bulletEngine->getFloor())
	{
		// standard box visu:
		/*
		SceneObjectPtr so = bulletEngine->getFloor()->getSceneObject();
		SoNode * n = CoinVisualizationFactory::getCoinVisualization(so,SceneObject::Full);
		*/

		// better grid visu
		Eigen::Vector3f floorPos;
		Eigen::Vector3f floorUp;
		float floorExtendMM;
		float floorDepthMM;
		bulletEngine->getFloorInfo(floorPos,floorUp,floorExtendMM,floorDepthMM);
		SoNode * n = (SoNode*)CoinVisualizationFactory::CreatePlaneVisualization(floorPos,floorUp,floorExtendMM,0.0f);
		if (n)
		{
			floor->addChild(n);
			addedVisualizations[bulletEngine->getFloor()] = n;
		}
		//addVisualization(bulletEngine->getFloor());
	}
	if (scene)
		sceneGraph->addChild(scene);

	viewer->setSceneGraph(sceneGraphRoot);
	viewer->viewAll();
}

void BulletCoinQtViewer::scheduleRedraw()
{
	sceneGraphRoot->touch();
}

void BulletCoinQtViewer::stepPhysics()
{
	boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	if (bulletEngine)
	{
		bulletEngine->activateAllObjects(); // avoid sleeping objects

		// Commented out: This is now handled by Bullet (bulletMaxSubSteps * bulletTimeStepMsec is the maximal duration of a frame)
		/* float minFPS = 1000000.f/40.f;  // Don't use 60 Hz (cannot be reached due to Vsync)
		if (ms > minFPS) {
			VR_INFO << "Slow frame (" << ms << "us elapsed)! Limiting elapsed time (losing realtime capabilities for this frame)." << endl;
			ms = minFPS;
		} */

		if ((ms / 1000.0f) > bulletMaxSubSteps * bulletTimeStepMsec) {
			VR_INFO << "Elapsed time (" << (ms / 1000.0f) << "ms) too long: Simulation is not running in realtime." << endl;
		}

		btScalar dt1 = btScalar(ms / 1000000.0f);

		// VR_INFO << "stepSimulation(" << dt1 << ", " << bulletMaxSubSteps << ", " << (bulletTimeStepMsec / 1000.0f) << ")" << endl;
		bulletEngine->stepSimulation(dt1, bulletMaxSubSteps, bulletTimeStepMsec / 1000.0f);

		//optional but useful: debug drawing
		//m_dynamicsWorld->debugDrawWorld();
	}
}


btScalar BulletCoinQtViewer::getDeltaTimeMicroseconds()
{
	btScalar dt = (btScalar)m_clock.getTimeMicroseconds();
	m_clock.reset();
	return dt;
}

void BulletCoinQtViewer::viewAll()
{

	viewer->getCamera()->viewAll(sceneGraph,viewer->getViewportRegion());
	//viewer->viewAll();
}

void BulletCoinQtViewer::addVisualization(DynamicsObjectPtr o, VirtualRobot::SceneObject::VisualizationType visuType)
{
	boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	VR_ASSERT(o);
	SceneObjectPtr so = o->getSceneObject();
	VR_ASSERT(so);
	removeVisualization(o);
	SoNode * n = CoinVisualizationFactory::getCoinVisualization(so,visuType);
	if (n)
	{
		sceneGraph->addChild(n);
		sceneGraph->addSelectionCallback( selectionCB, this );
		sceneGraph->addDeselectionCallback( deselectionCB, this );
		addedVisualizations[o] = n;
	}
}

void BulletCoinQtViewer::addStepCallback(BulletStepCallback callback, void* data)
{
	bulletEngine->addExternalCallback(callback, data);
}

void BulletCoinQtViewer::addVisualization(DynamicsRobotPtr r, VirtualRobot::SceneObject::VisualizationType visuType)
{
	boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	VR_ASSERT(r);
	RobotPtr ro = r->getRobot();
	VR_ASSERT(ro);
	removeVisualization(r);

	std::vector<RobotNodePtr> collectedRobotNodes;
	ro->getRobotNodes(collectedRobotNodes);
	std::vector<VisualizationNodePtr> collectedVisualizationNodes(collectedRobotNodes.size());
	for (size_t i=0;i<collectedRobotNodes.size();i++)
		collectedVisualizationNodes[i] = collectedRobotNodes[i]->getVisualization(visuType);

	SoSeparator* n = new SoSeparator();
	BOOST_FOREACH(VisualizationNodePtr visualizationNode, collectedVisualizationNodes)
	{
			boost::shared_ptr<CoinVisualizationNode> coinVisualizationNode = boost::dynamic_pointer_cast<CoinVisualizationNode>(visualizationNode);
			if (coinVisualizationNode && coinVisualizationNode->getCoinVisualization())
					n->addChild(coinVisualizationNode->getCoinVisualization());
	}
	sceneGraph->addChild(n);
	addedRobotVisualizations[r] = n;

	/*SoNode * n = CoinVisualizationFactory::getCoinVisualization(ro,visuType);
	if (n)
	{
		sceneGraph->addChild(n);
		addedRobotVisualizations[r] = n;
	}*/
}

void BulletCoinQtViewer::removeVisualization( DynamicsObjectPtr o )
{
	boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	VR_ASSERT(o);
	if (addedVisualizations.find(o) != addedVisualizations.end())
	{
		sceneGraph->removeChild(addedVisualizations[o]);
		addedVisualizations.erase(o);
	}
}

void BulletCoinQtViewer::removeVisualization( DynamicsRobotPtr r )
{
	boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	VR_ASSERT(r);
	if (addedRobotVisualizations.find(r) != addedRobotVisualizations.end())
	{
		sceneGraph->removeChild(addedRobotVisualizations[r]);
		addedRobotVisualizations.erase(r);
	}
}

void BulletCoinQtViewer::stopCB()
{
	boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	if (timerSensor)
	{
		SoSensorManager *sensor_mgr = SoDB::getSensorManager();
		sensor_mgr->removeTimerSensor(timerSensor);
		delete timerSensor;
		timerSensor = NULL;
	}
	if (sceneGraph)
	{
		sceneGraph->removeSelectionCallback(selectionCB,this);
		sceneGraph->removeDeselectionCallback(deselectionCB,this);
	}
}

void BulletCoinQtViewer::setBulletSimTimeStepMsec(int msec)
{
	boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	VR_ASSERT(msec > 0);
	bulletTimeStepMsec = msec;
}

void BulletCoinQtViewer::setBulletSimMaxSubSteps(int n)
{
	boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	VR_ASSERT(n > 0);
	bulletMaxSubSteps = n;
}

bool BulletCoinQtViewer::engineRunning()
{
	boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	return enablePhysicsUpdates;
}

void BulletCoinQtViewer::stopEngine()
{
	boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	enablePhysicsUpdates = false;
}
void BulletCoinQtViewer::startEngine()
{
	boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	enablePhysicsUpdates = true;
}

void BulletCoinQtViewer::updatePhysics()
{
	boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);

	if (enablePhysicsUpdates)
		stepPhysics();
}

}

