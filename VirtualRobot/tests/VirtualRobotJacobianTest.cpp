/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotJacobianTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

BOOST_AUTO_TEST_SUITE(RobotNode)

#define FLOAT_CLOSE_TO_DIFF 1e-7f


BOOST_AUTO_TEST_CASE(testJacobianRevoluteJoint)
{
	const std::string robotString =
		"<Robot Type='MyDemoRobotType' StandardName='ExampleRobo' RootNode='Joint1'>"
		" <RobotNode name='Joint1'>"

		"  <Joint type='revolute'>"
		"   <Limits unit='degree' lo='-45' hi='45'/>"
		"	<Axis x='1' y='0' z='0'/>"
		"   <PostJointTransform>"
		"    <Transform>"
		"     <Translation x='100' y='0' z='0'/>"
		"    </Transform>"
		"   </PostJointTransform>"
		"  </Joint>"
		"  <Child name='Joint2'/>"
		" </RobotNode>"

		" <RobotNode name='Joint2'>"
		"  <Joint type='revolute'>"
		"   <PreJointTransform>"
		"    <Transform>"
		"     <Translation x='0' y='200' z='0'/>"
		"    </Transform>"
		"   </PreJointTransform>"
		"   <Limits unit='degree' lo='-45' hi='45'/>"
		"	<Axis x='0' y='0' z='1'/>"
		"  </Joint>"
		"  <Child name='Joint3'/>"
		" </RobotNode>"

		" <RobotNode name='Joint3'>"
		"  <Joint type='fixed'>"
		"   <PreJointTransform>"
		"    <Transform>"
		"     <Translation x='0' y='200' z='0'/>"
		"    </Transform>"
		"   </PreJointTransform>"
		"  </Joint>"
		" </RobotNode>"
		"</Robot>";
	VirtualRobot::RobotPtr rob;
	BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
	BOOST_REQUIRE(rob);

	const std::string node1 = "Joint1";
	const std::string node2 = "Joint2";
	const std::string node3 = "Joint2";
	VirtualRobot::RobotNodePtr r1 = rob->getRobotNode(node1);
	VirtualRobot::RobotNodePtr r2 = rob->getRobotNode(node2);
	VirtualRobot::RobotNodePtr r3 = rob->getRobotNode(node3);
	BOOST_REQUIRE(r1);
	BOOST_REQUIRE(r2);
	BOOST_REQUIRE(r3);

	std::vector< VirtualRobot::RobotNodePtr > nodes;
	nodes.push_back(r1);
	nodes.push_back(r3);
	VirtualRobot::RobotNodeSetPtr kc(VirtualRobot::RobotNodeSet::createRobotNodeSet(rob,"KinChain",nodes,r1));
	BOOST_REQUIRE(kc);
	BOOST_CHECK_EQUAL(kc->isKinematicChain(),true);

	Eigen::VectorXf jV(2);
	jV << 0, 0; 
	kc->setJointValues(jV);
}

BOOST_AUTO_TEST_SUITE_END()
