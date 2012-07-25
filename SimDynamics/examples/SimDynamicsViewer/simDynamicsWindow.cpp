
#include "simDynamicsWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include <VirtualRobot/RuntimeEnvironment.h>

#include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoCube.h>

#include <sstream>
using namespace std;
using namespace VirtualRobot;

SimDynamicsWindow::SimDynamicsWindow(std::string &sRobotFilename, Qt::WFlags flags)
:QMainWindow(NULL)
{
	VR_INFO << " start " << endl;
	//this->setCaption(QString("ShowRobot - KIT - Humanoids Group"));
	//resize(1100, 768);

	useColModel = false;
	VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(sRobotFilename);
	robotFilename = sRobotFilename;
	sceneSep = new SoSeparator;
	sceneSep->ref();

	// optional visualizations (not considered by dynamics)
	SoSeparator *cc = CoinVisualizationFactory::CreateCoordSystemVisualization(10.0f);
	sceneSep->addChild(cc);

	dynamicsWorld = SimDynamics::DynamicsWorld::Init();
	SIMDYNAMICS_ASSERT(dynamicsWorld);

	dynamicsWorld->createFloorPlane();

	VirtualRobot::ObstaclePtr o = VirtualRobot::Obstacle::createBox(1000.0f,1000.0f,1000.0f,VirtualRobot::VisualizationFactory::Color::Blue());
	o->setMass(1.0f); // 1kg

	dynamicsObject = dynamicsWorld->CreateDynamicsObject(o);
	dynamicsObject->setPosition(Eigen::Vector3f(3000,3000,10000.0f));
	dynamicsWorld->addObject(dynamicsObject);

	loadRobot();

	setupUI();

	// build visualization
	collisionModel();

	viewer->viewAll();
}


SimDynamicsWindow::~SimDynamicsWindow()
{
	robot.reset();
	dynamicsRobot.reset();
	sceneSep->unref();
}


void SimDynamicsWindow::setupUI()
{
	UI.setupUi(this);

	viewer.reset(new SimDynamics::BulletCoinQtViewer(dynamicsWorld));
	viewer->initSceneGraph(UI.frameViewer,sceneSep);
	connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
	connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));


	/*connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));
	connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeHand()));
	connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openHand()));
	connect(UI.comboBoxEndEffector, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));



	connect(UI.checkBoxStructure, SIGNAL(clicked()), this, SLOT(robotStructure()));
	UI.checkBoxFullModel->setChecked(true);
	connect(UI.checkBoxFullModel, SIGNAL(clicked()), this, SLOT(robotFullModel()));
	connect(UI.checkBoxRobotCoordSystems, SIGNAL(clicked()), this, SLOT(robotCoordSystems()));
	connect(UI.checkBoxShowCoordSystem, SIGNAL(clicked()), this, SLOT(showCoordSystem()));
	connect(UI.comboBoxRobotNodeSet, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));
	connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
	connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));*/

}



void SimDynamicsWindow::resetSceneryAll()
{
	if (robot)
		robot->applyJointValues();
	//cout << "nyi..." << endl;
/*	if (!robot)
		return;
	std::vector<RobotNodePtr> allRobotNodes = robot->getRobotNodes();
	std::vector<float> jv(allRobotNodes.size(),0.0f);
	robot->setJointValues(allRobotNodes,jv);*/
}




void SimDynamicsWindow::collisionModel()
{
	if (!robot)
		return;

	useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
	SceneObject::VisualizationType colModel = useColModel?SceneObject::Collision:SceneObject::Full;
	viewer->addVisualization(dynamicsRobot,colModel);
	viewer->addVisualization(dynamicsObject,colModel);
}


void SimDynamicsWindow::closeEvent(QCloseEvent *event)
{
	quit();
	QMainWindow::closeEvent(event);
}




int SimDynamicsWindow::main()
{
	SoQt::show(this);
	SoQt::mainLoop();
	return 0;
}


void SimDynamicsWindow::quit()
{
	std::cout << "SimDynamicsWindow: Closing" << std::endl;
	this->close();
	SoQt::exitMainLoop();
}

/*void simDynamicsWindow::selectRobot()
{
	QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr("XML Files (*.xml)"));
	m_sRobotFilename = std::string(fi.toAscii());
	loadRobot();
}*/

void SimDynamicsWindow::loadRobot()
{
	cout << "Loading Robot from " << robotFilename << endl;

	try
	{
		robot = RobotIO::loadRobot(robotFilename,RobotIO::eFull);
	}
	catch (VirtualRobotException &e)
	{
		cout << " ERROR while creating robot" << endl;
		cout << e.what();
		return;
	}
	
	if (!robot)
	{
		cout << " ERROR while creating robot" << endl;
		return;
	}
	Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
	gp(2,3) = 20.0f;
	robot->setGlobalPose(gp);
	dynamicsRobot = dynamicsWorld->CreateDynamicsRobot(robot);
	dynamicsWorld->addRobot(dynamicsRobot);

}

