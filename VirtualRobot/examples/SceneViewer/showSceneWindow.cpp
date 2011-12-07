
#include "showSceneWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/ReachabilitySpace.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/XML/ObjectIO.h"

 #include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

showSceneWindow::showSceneWindow(std::string &sSceneFile, Qt::WFlags flags)
:QMainWindow(NULL)
{
	VR_INFO << " start " << endl;

	m_sSceneFile = sSceneFile;
	sceneSep = new SoSeparator;
	sceneSep->ref();
	sceneVisuSep = new SoSeparator;

	sceneSep->addChild(sceneVisuSep);

	setupUI();
	
	loadScene();

	m_pExViewer->viewAll();
}


showSceneWindow::~showSceneWindow()
{
	sceneSep->unref();
}


void showSceneWindow::setupUI()
{
	 UI.setupUi(this);
	 m_pExViewer = new SoQtExaminerViewer(UI.frameViewer,"",TRUE,SoQtExaminerViewer::BUILD_POPUP);

	// setup
	m_pExViewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
	m_pExViewer->setAccumulationBuffer(true);
#ifdef WIN32
#ifndef _DEBUG
	m_pExViewer->setAntialiasing(true, 4);
#endif
#endif
	m_pExViewer->setGLRenderAction(new SoLineHighlightRenderAction);
	m_pExViewer->setTransparencyType(SoGLRenderAction::BLEND);
	m_pExViewer->setFeedbackVisibility(true);
	m_pExViewer->setSceneGraph(sceneSep);
	m_pExViewer->viewAll();

	connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
	connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectScene()));


	/*connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeHand()));
	connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openHand()));
	connect(UI.comboBoxEndEffector, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));


	connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
	connect(UI.checkBoxStructure, SIGNAL(clicked()), this, SLOT(robotStructure()));
	UI.checkBoxFullModel->setChecked(true);
	connect(UI.checkBoxFullModel, SIGNAL(clicked()), this, SLOT(robotFullModel()));
	connect(UI.checkBoxRobotCoordSystems, SIGNAL(clicked()), this, SLOT(robotCoordSystems()));
	connect(UI.checkBoxShowCoordSystem, SIGNAL(clicked()), this, SLOT(showCoordSystem()));
	connect(UI.comboBoxRobotNodeSet, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));
	connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
	connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));*/

}

QString showSceneWindow::formatString(const char *s, float f)
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


void showSceneWindow::resetSceneryAll()
{
	/*if (!m_pRobot)
		return;

	for (unsigned int i=0;i<allRobotNodes.size();i++)
	{
		allRobotNodes[i]->setJointValue(0);
	}

	selectJoint(UI.comboBoxJoint->currentIndex());*/
}


/*
void showSceneWindow::displayTriangles()
{
	QString text1,text2,text3;
	int trisAllFull, trisRNSFull, trisJointFull;
	trisAllFull = trisRNSFull = trisJointFull = 0;
	int trisAllCol,trisRNSCol,trisJointCol;
	trisAllCol = trisRNSCol = trisJointCol = 0;
	if (m_pRobot)
	{
		trisAllFull = m_pRobot->getNumFaces(false);
		trisAllCol = m_pRobot->getNumFaces(true);
		trisRNSFull = trisAllFull;
		trisRNSCol = trisAllCol;
	}
	if (currentRobotNodeSet)
	{
		trisRNSFull = currentRobotNodeSet->getNumFaces(false);
		trisRNSCol = currentRobotNodeSet->getNumFaces(true);
	}
	if (currentRobotNode)
	{
		trisJointFull = currentRobotNode->getNumFaces(false);
		trisJointCol = currentRobotNode->getNumFaces(true);
	}
	if (UI.checkBoxColModel->checkState() == Qt::Checked)
	{
		text1 = tr("Total\t:") + QString::number(trisAllCol);
		text2 = tr("RobotNodeSet:\t") + QString::number(trisRNSCol);
		text3 = tr("Joint:\t") + QString::number(trisJointCol);
	} else
	{		
		text1 = tr("Total:\t") + QString::number(trisAllFull);
		text2 = tr("RobotNodeSet:\t") + QString::number(trisRNSFull);
		text3 = tr("Joint:\t") + QString::number(trisJointFull);
	}
	UI.labelInfo1->setText(text1);
	UI.labelInfo2->setText(text2);
	UI.labelInfo3->setText(text3);
}

void showSceneWindow::robotFullModel()
{
	if (!m_pRobot)
		return;

	bool showFullModel = UI.checkBoxFullModel->checkState() == Qt::Checked;

	m_pRobot->setupVisualization(showFullModel, true);

}

void showSceneWindow::collisionModel()
{
	if (!m_pRobot)
		return;
	robotSep->removeAllChildren();
	//setRobotModelShape(UI.checkBoxColModel->state() == QCheckBox::On);
	useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    visualization = m_pRobot->getVisualization<CoinVisualization>(useColModel);
	SoNode* visualisationNode = NULL;
	if (visualization)
		visualisationNode = visualization->getCoinVisualization();

	if (visualisationNode)
		robotSep->addChild(visualisationNode);

	selectJoint(UI.comboBoxJoint->currentIndex());

	UI.checkBoxStructure->setEnabled(!useColModel);
	UI.checkBoxFullModel->setEnabled(!useColModel);
	UI.checkBoxRobotCoordSystems->setEnabled(!useColModel);

}*/


void showSceneWindow::closeEvent(QCloseEvent *event)
{
	quit();
	QMainWindow::closeEvent(event);
}



void showSceneWindow::buildVisu()
{
	if (!scene)
		return;
	sceneVisuSep->removeAllChildren();
	//setRobotModelShape(UI.checkBoxColModel->state() == QCheckBox::On);
	//useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
	visualization = scene->getVisualization<CoinVisualization>();
	SoNode* visualisationNode = NULL;
	if (visualization)
		visualisationNode = visualization->getCoinVisualization();

	if (visualisationNode)
		sceneVisuSep->addChild(visualisationNode);

	/*selectJoint(UI.comboBoxJoint->currentIndex());

	UI.checkBoxStructure->setEnabled(!useColModel);
	UI.checkBoxFullModel->setEnabled(!useColModel);
	UI.checkBoxRobotCoordSystems->setEnabled(!useColModel);*/

}

int showSceneWindow::main()
{
	SoQt::show(this);
	SoQt::mainLoop();
	return 0;
}


void showSceneWindow::quit()
{
	std::cout << "showSceneWindow: Closing" << std::endl;
	this->close();
	SoQt::exitMainLoop();
}

/*
void showSceneWindow::updateJointBox()
{
	UI.comboBoxJoint->clear();

	for (unsigned int i=0;i<currentRobotNodes.size();i++)
	{
		UI.comboBoxJoint->addItem(QString(currentRobotNodes[i]->getName().c_str()));
	}
}

void showSceneWindow::updateRNSBox()
{
	UI.comboBoxRobotNodeSet->clear();
	UI.comboBoxRobotNodeSet->addItem(QString("<All>"));

	for (unsigned int i=0;i<robotNodeSets.size();i++)
	{
		UI.comboBoxRobotNodeSet->addItem(QString(robotNodeSets[i]->getName().c_str()));
	}
}

void showSceneWindow::selectRNS(int nr)
{
	currentRobotNodeSet.reset();
	cout << "Selecting RNS nr " << nr << endl;
	if (nr<=0)
	{
		// all joints
		currentRobotNodes = allRobotNodes;
	} else
	{
		nr--;
		if (nr>=(int)robotNodeSets.size())
			return;
		currentRobotNodeSet = robotNodeSets[nr];
		currentRobotNodes = currentRobotNodeSet->getAllRobotNodes();
	}
	updateJointBox();
	selectJoint(0);
	displayTriangles();
}

void showSceneWindow::selectJoint(int nr)
{
	currentRobotNode.reset();
	cout << "Selecting Joint nr " << nr << endl;
	if (nr<0 || nr>=(int)currentRobotNodes.size())
		return;
	currentRobotNode = currentRobotNodes[nr];
	currentRobotNode->print();
	float mi = currentRobotNode->getJointLimitLo();
	float ma = currentRobotNode->getJointLimitHi();
	QString qMin = QString::number(mi);
	QString qMax = QString::number(ma);
	UI.labelMinPos->setText(qMin);
	UI.labelMaxPos->setText(qMax);
	float j = currentRobotNode->getJointValue();
	UI.lcdNumberJointValue->display((double)j);
	if (fabs(ma-mi)>0 && (currentRobotNode->isTranslationalJoint() || currentRobotNode->isRotationalJoint()) )
	{
		UI.horizontalSliderPos->setEnabled(true);
		int pos = (int)((j-mi)/(ma-mi) * 1000.0f);
		UI.horizontalSliderPos->setValue(pos);
	} else
	{
		UI.horizontalSliderPos->setValue(500);
		UI.horizontalSliderPos->setEnabled(false);
	}
	if (currentRobotNodes[nr]->showCoordinateSystemState())
		UI.checkBoxShowCoordSystem->setCheckState(Qt::Checked);
	else
		UI.checkBoxShowCoordSystem->setCheckState(Qt::Unchecked);

    cout << "HIGHLIGHTING node " << currentRobotNodes[nr]->getName() << endl;

    if (visualization)
    {
        m_pRobot->highlight(visualization,false);
        currentRobotNode->highlight(visualization,true);
    }
	displayTriangles();
}

void showSceneWindow::jointValueChanged(int pos)
{	
	int nr = UI.comboBoxJoint->currentIndex();
	if (nr<0 || nr>=(int)currentRobotNodes.size())
		return;
	float fPos = currentRobotNodes[nr]->getJointLimitLo() + (float)pos / 1000.0f * (currentRobotNodes[nr]->getJointLimitHi() - currentRobotNodes[nr]->getJointLimitLo());
	currentRobotNodes[nr]->setJointValue(fPos);
	UI.lcdNumberJointValue->display((double)fPos);

}

void showSceneWindow::showCoordSystem()
{	
	float size = 0.75f;
	int nr = UI.comboBoxJoint->currentIndex();
	if (nr<0 || nr>=(int)currentRobotNodes.size())
		return;

	// first check if robot node has a visualization 


	currentRobotNodes[nr]->showCoordinateSystem(UI.checkBoxShowCoordSystem->checkState() == Qt::Checked, size);
	// rebuild visualization
	collisionModel();
}

*/

void showSceneWindow::selectScene()
{
	QString fi = QFileDialog::getOpenFileName(this, tr("Open Scene File"), QString(), tr("XML Files (*.xml)"));
	m_sSceneFile = std::string(fi.toAscii());
	loadScene();
}

void showSceneWindow::loadScene()
{
	sceneVisuSep->removeAllChildren();
	cout << "Loading Scene from " << m_sSceneFile << endl;

	try
	{
		scene = SceneIO::loadScene(m_sSceneFile);
	}
	catch (VirtualRobotException &e)
	{
		cout << " ERROR while creating scene" << endl;
		cout << e.what();
		return;
	}
	
	if (!scene)
	{
		cout << " ERROR while creating scene" << endl;
		return;
	}

	// get nodes
	/*m_pRobot->getRobotNodes(allRobotNodes);
	m_pRobot->getRobotNodeSets(robotNodeSets);
	m_pRobot->getEndEffectors(eefs);
	updateEEFBox();
	updateRNSBox();
	selectRNS(0);
	if (allRobotNodes.size()==0)
		selectJoint(-1);
	else
		selectJoint(0);

	if (eefs.size()==0)
		selectEEF(-1);
	else
		selectEEF(0);

	displayTriangles();

	// build visualization
	collisionModel();
	robotStructure();*/
	buildVisu();
	m_pExViewer->viewAll();
}

/*
void showSceneWindow::robotStructure()
{
	if (!m_pRobot)
		return;

	structureEnabled = UI.checkBoxStructure->checkState() == Qt::Checked;
	m_pRobot->showStructure(structureEnabled);
	// rebuild visualization
	collisionModel();
}

void showSceneWindow::robotCoordSystems()
{
	if (!m_pRobot)
		return;

	bool robtoAllCoordsEnabled = UI.checkBoxRobotCoordSystems->checkState() == Qt::Checked;
	m_pRobot->showCoordinateSystems(robtoAllCoordsEnabled);
	// rebuild visualization
	collisionModel();
}

*/
