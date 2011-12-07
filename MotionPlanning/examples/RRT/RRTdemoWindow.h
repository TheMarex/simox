
#ifndef __RRTdemo_WINDOW_H_
#define __RRTdemo_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <string.h>

#include <qobject.h>
#include <qmainwindow.h>
#include <qpushbutton.h>
#include <qtextedit.h>
#include <qcombobox.h>
#include <qprogressbar.h>
#include <qlabel.h>
#include <qslider.h>
#include <qcheckbox.h>
#include <qspinbox.h>
#include <qlineedit.h>

#include <vector>

#include "ui_RRTdemo.h"

class CRobot;
class SoQtExaminerViewer;
class SoSeparator;
class SoSensor;
class CRRTdemoScenery;

class CRRTdemoWindow : public QMainWindow
{
	Q_OBJECT
public:
	CRRTdemoWindow(Qt::WFlags flags = 0);
	~CRRTdemoWindow();

	/*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
	int main();

public slots:
	/*! Closes the window and exits SoQt runloop. */
	void quit();

	/*!< Overriding the close event, so we know when the window was closed by the user. */
	void closeEvent(QCloseEvent *event);

	void resetSceneryAll();
	void collisionModel();
	void showRobot();

	SoQtExaminerViewer* getExaminerViewer(){return m_pExViewer;};

protected:
	void setupUI();
	QString formatString(const char *s, float f);

	void loadRobot();

	Ui_MainWindowRRTDemo UI;
	SoQtExaminerViewer* m_pExViewer; /*!< Viewer to display the 3D model of the robot and the environment. */
		
	SoSeparator* m_pSceneSep;
	SoSeparator* m_pRobotSep;



};

#endif // __RRTdemo_WINDOW_H_
