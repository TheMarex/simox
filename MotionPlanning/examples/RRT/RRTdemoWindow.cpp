
#include "RRTdemoWindow.h"

#include "VirtualRobot/XML/RobotIO.h"
#include "VirtualRobot/CollisionDetection/CollisionChecker.h"
#include "VirtualRobot/Robot.h"

#include <time.h>
#include <vector>
#include <iostream>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <Qt3Support/Qt3Support>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>


#include <sstream>
using namespace std;

float TIMER_MS = 30.0f;

CRRTdemoWindow::CRRTdemoWindow(Qt::WFlags flags)
:QMainWindow(NULL)
{
	cout << __PRETTY_FUNCTION__ << " start " << endl;
	//this->setCaption(QString("RRTdemo - KIT - Humanoids Group"));
	//resize(1100, 768);


	m_pRobotSep = NULL;
	m_pGraspScenery = new CRRTdemoScenery();
	m_pSceneSep = m_pGraspScenery->getScene();

	setupUI();

	loadRobot();		// load robot before loading object so they are correctly linked
	//setEnvironment(0);
	//loadObject(0);

	updateGraspObjectPose(0,0,0,0,0,0);
	m_pExViewer->viewAll();
}


CRRTdemoWindow::~CRRTdemoWindow()
{
}

/*
void CRRTdemoWindow::saveScreenshot()
{
	static int counter = 0;
	SbString framefile;

	framefile.sprintf("MPL_Render_Frame%06d.png", counter);
	counter++;

	m_pExViewer->getSceneManager()->render();
	m_pExViewer->getSceneManager()->scheduleRedraw();
	QGLWidget* w = (QGLWidget*)m_pExViewer->getGLWidget();

	QImage i = w->grabFrameBuffer();
	bool bRes = i.save(framefile.getString(), "PNG");
	if (bRes)
		cout << "wrote image " << counter << endl;
	else
		cout << "failed writing image " << counter << endl;

}*/

void CRRTdemoWindow::setupUI()
{
	 UI.setupUi(this);
	 //centralWidget()->setLayout(UI.gridLayoutViewer);
	m_pExViewer = new SoQtExaminerViewer(UI.frameViewer,"",TRUE,SoQtExaminerViewer::BUILD_POPUP);

	// setup
	m_pExViewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
	m_pExViewer->setAccumulationBuffer(true);
#ifdef WIN32
#ifndef _DEBUG
	m_pExViewer->setAntialiasing(true, 4);
#endif
#endif
	m_pExViewer->setTransparencyType(SoGLRenderAction::BLEND);
	m_pExViewer->setFeedbackVisibility(true);
	m_pExViewer->setSceneGraph(m_pSceneSep);
	m_pExViewer->viewAll();

	connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
	connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
	//connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
	//connect(UI.horizontalSliderJointValue, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));

}

QString CRRTdemoWindow::formatString(const char *s, float f)
{
	QString str1(s);
	if (f>=0)
		str1 += " ";
	if (fabs(f)<1000)
		str1 += " ";
	if (fabs(f)<100)
		str1 += " ";
	if (fabs(f)<10)
		str1 += " ";
	QString str1n;
	str1n.setNum(f,'f',3);
	str1 = str1 + str1n;
	return str1;
}


void CRRTdemoWindow::resetSceneryAll()
{
	loadRobot();
}



void CRRTdemoWindow::collisionModel()
{
	//setRobotModelShape(UI.checkBoxColModel->state() == QCheckBox::On);
}


void CRRTdemoWindow::showRobot()
{
	//m_pGraspScenery->showRobot(m_pShowRobot->state() == QCheckBox::On);
}

void CRRTdemoWindow::closeEvent(QCloseEvent *event)
{
	quit();
	QMainWindow::closeEvent(event);
}




int CRRTdemoWindow::main()
{
	SoQt::show(this);
	SoQt::mainLoop();
	return 0;
}


void CRRTdemoWindow::quit()
{
	std::cout << "CRRTdemoWindow: Closing" << std::endl;
	this->close();
	SoQt::exitMainLoop();
}


void CRRTdemoWindow::loadRobot()
{
	std::cout << "CRRTdemoWindow: Loading robot" << std::endl;
	//m_pGraspScenery->loadRobot();
	m_pExViewer->viewAll();
}

