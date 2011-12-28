#include <QtCore/QTimer>
#include <QtGui/QApplication>
#include <QtGui/QGridLayout>
#include <QtGui/QKeyEvent>

#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>

#include <osgDB/ReadFile>

#include <osgQt/GraphicsWindowQt>

#include <iostream>

#include <QGLWidget>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>

class osgViewerWidget: public QGLWidget
{
	Q_OBJECT

	public:
		explicit osgViewerWidget(osg::Node* scene, QWidget *parent = 0);

	public slots:

		void timerCB();
	
	protected:
		virtual void initializeGL();
		virtual void resizeGL( int width, int height );

		virtual void paintGL();

		void resizeEvent(QResizeEvent *event);

		virtual void keyPressEvent( QKeyEvent* event );
		virtual void keyReleaseEvent( QKeyEvent* event );
		virtual void mousePressEvent( QMouseEvent* event );
		virtual void mouseReleaseEvent( QMouseEvent* event );
		virtual void mouseMoveEvent( QMouseEvent* event );
		virtual void mouseDoubleClickEvent(QMouseEvent* event);
		virtual void wheelEvent(QWheelEvent* event);

		void paintOSG();
		void resizeOSG(int width, int height);
protected:

		osg::ref_ptr<osgViewer::Viewer> viewer;
		osg::observer_ptr<osgViewer::GraphicsWindowEmbedded> window;
		osg::ref_ptr<osg::Node> loadedModel;
		osg::ref_ptr<osg::MatrixTransform> transformation;

		//osgQt::GraphicsWindowQt* m_qt_win;

		QTimer paintTimer;
		QSize canvasSize;
};

/*
class osgViewerWidget : public QWidget, public osgViewer::CompositeViewer
{
public:
	osgViewerWidget(osg::Node* scene, QWidget* parent = NULL, osgViewer::ViewerBase::ThreadingModel threadingModel=osgViewer::CompositeViewer::SingleThreaded);



	osg::Camera* createCamera( int x, int y, int w, int h, const std::string& name="", bool windowDecoration=false );


	virtual void paintEvent( QPaintEvent* event );
	QWidget* getQWidget();

protected:
	osgViewer::View* view;
	osg::Camera* camera;
	QTimer _timer;
};*/


