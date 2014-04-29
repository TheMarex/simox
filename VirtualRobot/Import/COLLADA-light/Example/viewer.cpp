#include "../inventor.h"
#include <boost/foreach.hpp>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/actions/SoWriteAction.h>

using namespace std;
using namespace Collada;

int main(int argc, char **argv ){
    QWidget * mainwin = SoQt::init(argc, argv, argv[0]);

    SoSeparator *root = new SoSeparator;
    root->ref();
    InventorRobot robot(root);

    if (argc==1)
        robot.parse("../RobotEditorArmar4.dae");
    else
        robot.parse(argv[1]);

    SoWriteAction writeAction;
    writeAction.getOutput()->openFile("test.iv");
    writeAction.apply(root);

    SoQtExaminerViewer * viewer = new SoQtExaminerViewer(mainwin);
    viewer->setSceneGraph(root);
    viewer->show();

    // Pop up the main window.
    SoQt::show(mainwin);
    // Loop until exit.
    SoQt::mainLoop();

    root->unref();
}
