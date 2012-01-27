/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotRobotTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <string>

BOOST_AUTO_TEST_SUITE(RobotFactory)

BOOST_AUTO_TEST_CASE(testRobotFactoryEmptyXML)
{
	const std::string robotString = "";
	BOOST_REQUIRE_THROW((VirtualRobot::RobotIO::createRobotFromString(robotString)), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryUnclosedRobotTag)
{
	const std::string robotString = "<Robot>";
	VirtualRobot::RobotPtr robot;
	BOOST_REQUIRE_THROW(robot = VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryOnlyClosedRobotTag)
{
	const std::string robotString = "</Robot>";
	BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryEmptyRobotTag)
{
	const std::string robotString = "<Robot></Robot>";
	BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryEmptyTypeString)
{
	const std::string robotString = "<Robot Type=''></Robot>";
	BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryNotExistentType)
{
	const std::string robotString = "<Robot Type='XYZ'></Robot>";
	BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotRobotMacro)
{
	const std::string robotString =
		"<Robot Type='MyDemoRobotType' StandardName='ExampleRobo' RootNode='Joint1'>"
		" <RobotNode name='Joint1'>"
		"  <Visualization enable='true'>"
		"  </Visualization>"
		" </RobotNode>"
		"</Robot>";
	VirtualRobot::RobotPtr rob;
	BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
	BOOST_REQUIRE(rob);

	const std::string node = "Joint1";
	VirtualRobot::RobotNodePtr r1 = rob->getRobotNode(node);
	BOOST_REQUIRE(r1);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryEmptyRootNodeString)
{
	const std::string robotString = "<Robot RootNode=''></Robot>";
	BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryNotExistentRootNode)
{
	const std::string robotString = "<Robot RootNode='JointX'></Robot>";
	BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotValidEndeffector)
{
	const std::string robotString =
		"<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
		" <RobotNode name='Joint1'>"
		"  <Visualization enable='true'>"
		"  </Visualization>"
		"  <Child name='Joint2'/>"
		" </RobotNode>"
		" <RobotNode name='Joint2'>"
		"  <Visualization enable='true'>"
		"  </Visualization>"
		"  <Child name='Joint3'/>"
		" </RobotNode>"
		" <RobotNode name='Joint3'>"
		"  <Visualization enable='true'>"
		"  </Visualization>"
		"  <Child name='Joint4'/>"
		" </RobotNode>"
		" <RobotNode name='Joint4'>"
		"  <Visualization enable='true'>"
		"  </Visualization>"
		" </RobotNode>"
		" <Endeffector name='endeffector1' base='Joint1' tcp='Joint1'>"
		"  <Static>"
		"   <Node name='Joint1'/>"
		"   <Node name='Joint2'/>"
		"  </Static>"
		"  <Actor name='actor1'>"
		"   <Node name='Joint3'/>"
		"   <Node name='Joint4'/>"
		"  </Actor>"
		" </Endeffector>"
		"</Robot>";
	VirtualRobot::RobotPtr rob;
	BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
	BOOST_REQUIRE(rob);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotInvariantTagPosition)
{
	// test if references to nodes are resolved correctly if the
	// nodes are defined after they are referenced
	const std::string robotString =
		"<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
		" <RobotNode name='Joint3'>"
		"  <Visualization enable='true'>"
		"  </Visualization>"
		"  <Child name='Joint4'/>"
		" </RobotNode>"
		" <RobotNode name='Joint4'>"
		"  <Visualization enable='true'>"
		"  </Visualization>"
		" </RobotNode>"
		" <Endeffector name='endeffector1' base='Joint1' tcp='Joint1'>"
		"  <Static>"
		"   <Node name='Joint1'/>"
		"   <Node name='Joint2'/>"
		"  </Static>"
		"  <Actor name='actor1'>"
		"   <Node name='Joint3'/>"
		"   <Node name='Joint4'/>"
		"  </Actor>"
		" </Endeffector>"
		" <RobotNode name='Joint1'>"
		"  <Visualization enable='true'>"
		"  </Visualization>"
		"  <Child name='Joint2'/>"
		" </RobotNode>"
		" <RobotNode name='Joint2'>"
		"  <Visualization enable='true'>"
		"  </Visualization>"
		"  <Child name='Joint3'/>"
		" </RobotNode>"
		"</Robot>";
	VirtualRobot::RobotPtr rob;
	BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
	BOOST_REQUIRE(rob);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotEndeffectorWrongChildTag)
{
	const std::string robotString =
		"<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
		" <RobotNode name='Joint1'>"
		"  <Visualization enable='true'>"
		"   <CoordinateAxis enable='true' scaling='1' text='Axis1'/>"
		"  </Visualization>"
		" </RobotNode>"
		" <Endeffector name='endeffector1' base='Joint1' tcp='Joint1'>"
		"  <XYZ>"
		"  </XYZ>"
		"  <Static>"
		"   <Node name='Joint1'/>"
		"  </Static>"
		" </Endeffector>"
		"</Robot>";
	BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), std::exception);//VirtualRobot::VirtualRobotException);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotEndeffectorWithoutNameTag)
{
	const std::string robotString =
		"<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
		" <Endeffector base='Joint1' tcp='Joint1'>"
		" </Endeffector>"
		"</Robot>";
	BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotEndeffectorMissingBasenodeTag)
{
	const std::string robotString =
		"<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
		" <Endeffector name ='e1'>"
		" </Endeffector>"
		"</Robot>";
	BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotEndeffectorMissingBasenode)
{
	const std::string robotString =
		"<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
		" <Endeffector name ='e1' base='Joint1' tcp='Joint1'>"
		" </Endeffector>"
		"</Robot>";
	BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotPhysicsTag)
{
	const std::string robotString =
		"<Robot Type='DemoRobotType' RootNode='Joint1'>"
		" <RobotNode name='Joint1'>"
		"  <Physics>"
		"   <Mass value='100' units='kg'/>"
		"   <CoM location='joint' x='10' y='20' z='30' units='mm'/>"
		"   <MaxVelocity value='0.3'/>"
		"   <MaxAcceleration value='0.1'/>"
		"   <IntertiaMatrix>"
		"     <row1 c1='1' c2='2' c3='3'/>"
		"     <row2 c1='4' c2='5' c3='6'/>"
		"     <row3 c1='7' c2='8' c3='9'/>"
		"   </IntertiaMatrix>"
		"  </Physics>"
		" </RobotNode>"
		"</Robot>";
	VirtualRobot::RobotPtr rob;
	BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
	BOOST_REQUIRE(rob);
	VirtualRobot::RobotNodePtr rn = rob->getRobotNode("Joint1");
	BOOST_REQUIRE(rn);
	float mass = rn->getMass();
	BOOST_CHECK_EQUAL(mass,100.0f);
	float vel = rn->getMaxVelocity();
	BOOST_CHECK_EQUAL(vel,0.3f);
	float acc = rn->getMaxAcceleration();
	BOOST_CHECK_EQUAL(acc,0.1f);
	Eigen::Vector3f com = rn->getCoMLocal();
	bool comOK = com.isApprox(Eigen::Vector3f(10.0f,20.0f,30.0f));
	BOOST_REQUIRE(comOK);

	Eigen::Matrix3f inertia = rn->getInertiaMatrix();
	Eigen::Matrix3f expectedMat;
	expectedMat << 1,2,3,4,5,6,7,8,9;
	bool inertiaMatrixOK = inertia.isApprox(expectedMat);
	BOOST_REQUIRE(inertiaMatrixOK);
}

BOOST_AUTO_TEST_SUITE_END()
