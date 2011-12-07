
#ifndef __ShowScene_WINDOW_H_
#define __ShowScene_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>


#include <vector>

#include "ui_SceneViewer.h"

class showSceneWindow : public QMainWindow
{
	Q_OBJECT
public:
	showSceneWindow(std::string &sSceneFile, Qt::WFlags flags = 0);
	~showSceneWindow();

	/*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
	int main();

public slots:
	/*! Closes the window and exits SoQt runloop. */
	void quit();

	/*!< Overriding the close event, so we know when the window was closed by the user. */
	void closeEvent(QCloseEvent *event);

	void resetSceneryAll();
	//void collisionModel();
	//void showRobot();
	void loadScene();
	/*void selectJoint(int nr);
	void selectRNS(int nr);
	void jointValueChanged(int pos);
	void showCoordSystem();
	void robotStructure();
	void robotCoordSystems();
	void robotFullModel();
	void closeHand();
	void openHand();
	void selectEEF(int nr);*/
	void selectScene();


	SoQtExaminerViewer* getExaminerViewer(){return m_pExViewer;};

protected:
	void setupUI();
	QString formatString(const char *s, float f);
	void buildVisu();
	/*void updateJointBox();
	void updateRNSBox();
	void updateEEFBox();
	void displayTriangles();*/
	Ui::MainWindowShowScene UI;
	SoQtExaminerViewer *m_pExViewer; /*!< Viewer to display the 3D model of the robot and the environment. */
		
	SoSeparator *sceneSep;
	SoSeparator *sceneVisuSep;

	VirtualRobot::ScenePtr scene;
	std::string m_sSceneFile;
	/*std::vector < VirtualRobot::RobotNodePtr > allRobotNodes;
	std::vector < VirtualRobot::RobotNodePtr > currentRobotNodes;
	std::vector < VirtualRobot::RobotNodeSetPtr > robotNodeSets;
	std::vector < VirtualRobot::EndEffectorPtr > eefs;
	VirtualRobot::EndEffectorPtr currentEEF;
	VirtualRobot::RobotNodeSetPtr currentRobotNodeSet;
	VirtualRobot::RobotNodePtr currentRobotNode;*/


	//bool useColModel;
	//bool structureEnabled;

    boost::shared_ptr<VirtualRobot::CoinVisualization> visualization;
};

#endif // __ShowScene_WINDOW_H_
