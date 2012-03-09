/**
* @package    VirtualRobot
* @author    Nikolaus Vahrenkamp
* @copyright  2012 nv
*/

#define BOOST_TEST_MODULE VirtualRobot_VoxelTreeTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Workspace/VoxelTree6D.hpp>

BOOST_AUTO_TEST_SUITE(VoxelTree)

BOOST_AUTO_TEST_CASE(VoxelTreeConstructionTest)
{
	float minB[6];
	float maxB[6];
	for (int i=0;i<3;i++)
	{
		minB[i] = -100.0f;
		maxB[i] = 100.0f;
		minB[3+i] = (float)-M_PI;
		maxB[3+i] = (float)M_PI;
	}
	BOOST_REQUIRE_NO_THROW(VirtualRobot::VoxelTree6D<unsigned char> v(minB,maxB,20.0f,1.0f));
}


BOOST_AUTO_TEST_CASE(VoxelTreeEntriesTest)
{
	float minB[6];
	float maxB[6];
	for (int i=0;i<3;i++)
	{
		minB[i] = -100.0f;
		maxB[i] = 100.0f;
		minB[3+i] = (float)-M_PI;
		maxB[3+i] = (float)M_PI;
	}
	VirtualRobot::VoxelTree6D<unsigned char> v(minB,maxB,20.0f,1.0f);
	float pos[6];
	for (int i=0;i<6;i++)
		pos[i] = 0;

	unsigned char* c = v.getEntry(pos);
	bool isNull = (c==NULL);
	BOOST_REQUIRE(isNull);

	v.setEntry(pos,10);
	c = v.getEntry(pos);
	isNull = (c==NULL);
	BOOST_REQUIRE(!isNull);

	BOOST_REQUIRE_EQUAL(*c,10);

}

BOOST_AUTO_TEST_SUITE_END()
