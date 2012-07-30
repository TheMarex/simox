
#include "simDynamicsWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>

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

	contactsSep = new SoSeparator;
	sceneSep->addChild(contactsSep);

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

	setupUI();
	loadRobot();

	// build visualization
	collisionModel();

	viewer->viewAll();

	// register callback
	float TIMER_MS = 30.0f;
	SoSensorManager *sensor_mgr = SoDB::getSensorManager();
	timerSensor = new SoTimerSensor(timerCB, this);
	timerSensor->setInterval(SbTime(TIMER_MS/1000.0f));
	sensor_mgr->insertTimerSensor(timerSensor);
}


SimDynamicsWindow::~SimDynamicsWindow()
{
	stopCB();
	dynamicsWorld.reset();
	SimDynamics::DynamicsWorld::Close();
	robot.reset();
	dynamicsRobot.reset();
	sceneSep->unref();
}

void SimDynamicsWindow::timerCB(void * data, SoSensor * sensor)
{
	SimDynamicsWindow *window = static_cast<SimDynamicsWindow*>(data);
	VR_ASSERT(window);

	// now its safe to update physical information and set the models to the according poses
	window->updateJointInfo();

	window->updateContactVisu();

}


void SimDynamicsWindow::setupUI()
{
	UI.setupUi(this);

	viewer.reset(new SimDynamics::BulletCoinQtViewer(dynamicsWorld));
	viewer->initSceneGraph(UI.frameViewer,sceneSep);
	connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
	connect(UI.checkBoxActuation, SIGNAL(clicked()), this, SLOT(actuation()));
	connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
	connect(UI.comboBoxRobotNode, SIGNAL(activated(int)), this, SLOT(selectRobotNode(int)));
	connect(UI.horizontalSliderTarget, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));

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



void SimDynamicsWindow::actuation()
{
	if (!dynamicsRobot)
		return;

	bool actuate = UI.checkBoxActuation->checkState() == Qt::Checked;

	if (actuate)
		dynamicsRobot->enableActuation();
	else
		dynamicsRobot->disableActuation();
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
	stopCB();
	this->close();
	SoQt::exitMainLoop();
}

/*void simDynamicsWindow::selectRobot()
{
	QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr("XML Files (*.xml)"));
	m_sRobotFilename = std::string(fi.toAscii());
	loadRobot();
}*/

void SimDynamicsWindow::updateJoints()
{
	if (!robot)
	{
		cout << " ERROR while creating list of nodes" << endl;
		return;
	}
	robotNodes.clear();
	UI.comboBoxRobotNode->clear();
	std::vector<RobotNodePtr> nodes = robot->getRobotNodes();
	for (size_t i=0;i<nodes.size();i++)
	{
		if (nodes[i]->isRotationalJoint())
		{
			RobotNodeRevolutePtr rn = boost::dynamic_pointer_cast<RobotNodeRevolute>(nodes[i]);
			if (rn)
			{
				robotNodes.push_back(rn);
				QString qstr(rn->getName().c_str());
				UI.comboBoxRobotNode->addItem(qstr);
			}
		}
	}
	if (robotNodes.size()>0)
		selectRobotNode(0);
	else
		selectRobotNode(-1);
}

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

	updateJoints();

}

void SimDynamicsWindow::selectRobotNode( int n )
{
	UI.comboBoxRobotNode->setCurrentIndex(n);
	RobotNodeRevolutePtr rn;
	if (n>=0 && n<(int)robotNodes.size())
	{	
		rn = robotNodes[n];
	}
	if (rn)
	{
		int pos = 0;
		if (rn->getJointValue()<0 && rn->getJointLimitLo()<0)
		{
			pos = int(rn->getJointValue()/rn->getJointLimitLo() * 100.0f + 0.5f);
		}
		if (rn->getJointValue()>0 && rn->getJointLimitHi()>0)
		{
			pos = int(rn->getJointValue()/rn->getJointLimitHi() * 100.0f + 0.5f);
		}
		UI.horizontalSliderTarget->setValue(pos);
		UI.horizontalSliderTarget->setEnabled(true);
	} else
	{
		UI.horizontalSliderTarget->setValue(0);
		UI.horizontalSliderTarget->setEnabled(false);
	}
}

void SimDynamicsWindow::updateJointInfo()
{
	int n = UI.comboBoxRobotNode->currentIndex();
	QString qMin("0");
	QString qMax("0");
	QString qName("Name: <not set>");
	QString qJV("Joint value: 0");
	QString qTarget("Joint target: 0");
	QString qVel("Joint velocity: 0");

	RobotNodeRevolutePtr rn;
	if (n>=0 && n<(int)robotNodes.size())
	{	
		rn = robotNodes[n];
	}
	//SimDynamics::DynamicsObjectPtr dynRN = dynamicsRobot->getDynamicsRobotNode(rn);
	if (rn)
	{
		qMin = QString::number(rn->getJointLimitLo(),'g',4);
		qMax = QString::number(rn->getJointLimitHi(),'g',4);
		qName = QString("Name: ");
		qName += QString(rn->getName().c_str());
		qJV = QString("Joint value: ");
		qJV += QString::number(rn->getJointValue(),'g',3);
		qJV += QString(" / ");
		qJV += QString::number(dynamicsRobot->getJointAngle(rn),'g',3);
		qTarget = QString("Joint target: ");
		qTarget += QString::number(dynamicsRobot->getNodeTarget(rn),'g',4);
		qVel = QString("Joint velocity: ");
		qVel += QString::number(dynamicsRobot->getJointSpeed(rn),'g',4);
		
	} 
	UI.label_TargetMin->setText(qMin);
	UI.label_TargetMax->setText(qMax);
	UI.label_RNName->setText(qName);
	UI.label_RNValue->setText(qJV);
	UI.label_RNTarget->setText(qTarget);
	UI.label_RNVelocity->setText(qVel);
}

void SimDynamicsWindow::jointValueChanged( int n )
{
	int j = UI.comboBoxRobotNode->currentIndex();
	RobotNodeRevolutePtr rn;
	if (j>=0 && j<(int)robotNodes.size())
	{	
		rn = robotNodes[j];
	}
	if (!rn || !dynamicsRobot)
		return;

	float pos = 0;
	if (n<0)
	{
		pos = (float(n) / 100.0f * fabs(rn->getJointLimitLo()));
	}
	if (n>0)
	{
		pos = (float(n) / 100.0f * fabs(rn->getJointLimitHi()));
	}
	dynamicsRobot->actuateNode(rn,pos);

}

void SimDynamicsWindow::stopCB()
{
	if (timerSensor)
	{
		SoSensorManager *sensor_mgr = SoDB::getSensorManager();
		sensor_mgr->removeTimerSensor(timerSensor);
		delete timerSensor;
		timerSensor = NULL;
	}
	viewer.reset();
}

void SimDynamicsWindow::updateContactVisu()
{
	contactsSep->removeAllChildren();
	if (!UI.checkBoxContacts->isChecked())
		return;
	std::vector<SimDynamics::DynamicsEngine::DynamicsContactInfo> c = dynamicsWorld->getEngine()->getContacts();
	for (size_t i=0;i<c.size();i++)
	{
		SoSeparator *normal = new SoSeparator;
		SoMatrixTransform *m = new SoMatrixTransform;
		SbMatrix ma;
		ma.makeIdentity();
		ma.setTranslate(SbVec3f(c[i].posGlobalB(0),c[i].posGlobalB(1),c[i].posGlobalB(2)));
		m->matrix.setValue(ma);
		normal->addChild(m);
		SoSeparator *n = CoinVisualizationFactory::CreateArrow(c[i].normalGlobalB,50.0f);
		if (n)
			normal->addChild(n);
		SoSeparator *normal2 = new SoSeparator;
		SoMatrixTransform *m2 = new SoMatrixTransform;
		ma.makeIdentity();
		ma.setTranslate(SbVec3f(c[i].posGlobalA(0),c[i].posGlobalA(1),c[i].posGlobalA(2)));
		m2->matrix.setValue(ma);
		normal2->addChild(m2);
		SoSeparator *n2 = CoinVisualizationFactory::CreateArrow(-c[i].normalGlobalB,50.0f);
		if (n2)
			normal2->addChild(n2);
		contactsSep->addChild(normal);
		contactsSep->addChild(normal2);
	}
}

