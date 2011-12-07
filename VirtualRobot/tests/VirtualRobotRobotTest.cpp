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
		"   <CoordinateAxis Type='Inventor' enable='true' scaling='1' text='Axis1'/>"
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
		"   <CoordinateAxis Type='Inventor' enable='true' scaling='1' text='Axis1'/>"
		"  </Visualization>"
		"  <Child name='Joint2'/>"
		" </RobotNode>"
		" <RobotNode name='Joint2'>"
		"  <Visualization enable='true'>"
		"   <CoordinateAxis type='Inventor' enable='true' scaling='1' text='Axis2'/>"
		"  </Visualization>"
		"  <Child name='Joint3'/>"
		" </RobotNode>"
		" <RobotNode name='Joint3'>"
		"  <Visualization enable='true'>"
		"   <CoordinateAxis type='Inventor' enable='true' scaling='1' text='Axis3'/>"
		"  </Visualization>"
		"  <Child name='Joint4'/>"
		" </RobotNode>"
		" <RobotNode name='Joint4'>"
		"  <Visualization enable='true'>"
		"   <CoordinateAxis type='Inventor' enable='true' scaling='1' text='Axis4'/>"
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
		"   <CoordinateAxis type='Inventor' enable='true' scaling='1' text='Axis3'/>"
		"  </Visualization>"
		"  <Child name='Joint4'/>"
		" </RobotNode>"
		" <RobotNode name='Joint4'>"
		"  <Visualization enable='true'>"
		"   <CoordinateAxis type='Inventor' enable='true' scaling='1' text='Axis4'/>"
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
		"   <CoordinateAxis Type='Inventor' enable='true' scaling='1' text='Axis1'/>"
		"  </Visualization>"
		"  <Child name='Joint2'/>"
		" </RobotNode>"
		" <RobotNode name='Joint2'>"
		"  <Visualization enable='true'>"
		"   <CoordinateAxis type='Inventor' enable='true' scaling='1' text='Axis2'/>"
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

BOOST_AUTO_TEST_SUITE_END()
