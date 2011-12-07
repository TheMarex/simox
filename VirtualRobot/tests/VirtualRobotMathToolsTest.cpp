/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotMathToolsTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <string>

BOOST_AUTO_TEST_SUITE(MathTools)

BOOST_AUTO_TEST_CASE(testMathToolsRPY)
{
	float r = (float)M_PI*0.25f;
	float p = 0;
	float y = (float)M_PI * 0.5f;
	Eigen::Matrix4f m;
	BOOST_REQUIRE_NO_THROW(VirtualRobot::MathTools::rpy2eigen4f(r,p,y,m));
	float x[6];
	BOOST_REQUIRE_NO_THROW(VirtualRobot::MathTools::eigen4f2rpy(m,x));

	BOOST_CHECK_EQUAL(0,x[0]);
	BOOST_CHECK_EQUAL(0,x[1]);
	BOOST_CHECK_EQUAL(0,x[2]);
	BOOST_CHECK_EQUAL(r,x[3]);
	BOOST_CHECK_EQUAL(p,x[4]);
	BOOST_CHECK_EQUAL(y,x[5]);
}

BOOST_AUTO_TEST_CASE(testMathToolsPlane)
{
	VirtualRobot::MathTools::Plane plane = VirtualRobot::MathTools::getFloorPlane();
	Eigen::Vector3f p(100.0f,200.0f,300.0f);
	Eigen::Vector3f res;
	BOOST_REQUIRE_NO_THROW(res = VirtualRobot::MathTools::projectPointToPlane(p,plane));


	BOOST_CHECK_EQUAL(res(0),p(0));
	BOOST_CHECK_EQUAL(res(1),p(1));
	BOOST_CHECK_EQUAL(res(2),0);

}


BOOST_AUTO_TEST_CASE(testMathToolsConvexHull2D)
{
	Eigen::Vector2f a(1.0f,1.0f);
	Eigen::Vector2f b(1.0f,2.0f);
	Eigen::Vector2f c(2.0f,1.0f);
	Eigen::Vector2f d(2.0f,2.0f);
	Eigen::Vector2f e(1.5f,1.5f);
	Eigen::Vector2f f(1.2f,1.8f);
	Eigen::Vector2f g(2.5f,2.5f);
	Eigen::Vector2f h(-1.5f,-1.5f);
	std::vector< Eigen::Vector2f > points;
	points.push_back(a);
	points.push_back(b);
	points.push_back(c);
	points.push_back(d);
	points.push_back(e);
	points.push_back(f);

	VirtualRobot::MathTools::ConvexHull2DPtr cv;
	BOOST_REQUIRE_NO_THROW( cv = VirtualRobot::MathTools::createConvexHull2D(points) );

	BOOST_REQUIRE(cv);

	bool isInsideE;
	bool isInsideF;
	bool isInsideG;
	bool isInsideH;
	BOOST_REQUIRE_NO_THROW( isInsideE = VirtualRobot::MathTools::isInside(e, cv) );
	BOOST_REQUIRE_NO_THROW( isInsideF = VirtualRobot::MathTools::isInside(f, cv) );
	BOOST_REQUIRE_NO_THROW( isInsideG = VirtualRobot::MathTools::isInside(g, cv) );
	BOOST_REQUIRE_NO_THROW( isInsideH = VirtualRobot::MathTools::isInside(h, cv) );

	BOOST_CHECK_EQUAL(isInsideE,true);
	BOOST_CHECK_EQUAL(isInsideF,true);
	BOOST_CHECK_EQUAL(isInsideG,false);
	BOOST_CHECK_EQUAL(isInsideH,false);
}


BOOST_AUTO_TEST_SUITE_END()
