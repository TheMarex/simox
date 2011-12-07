/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Peter Kaiser, Nikolaus Vahrenkamp
* @copyright  2011 Peter Kaiser, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_ReachabilitySpace_h_
#define _VirtualRobot_ReachabilitySpace_h_

#include "VirtualRobotImportExport.h"
#include "Visualization/VisualizationFactory.h"

#include <boost/enable_shared_from_this.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/mpl/assert.hpp>

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace VirtualRobot
{
class ReachabilitySpace;
/*!
	Stores a 6-dimensional array for the vertex data of a reachability space.
*/
class VIRTUAL_ROBOT_IMPORT_EXPORT ReachabilitySpaceData : public boost::enable_shared_from_this<ReachabilitySpaceData>
{
public:
	friend class ReachabilitySpace;
	/*!
		Constructor, fills the data with 0
	*/
	ReachabilitySpaceData(unsigned int size1, unsigned int size2, unsigned int size3,
						  unsigned int size4, unsigned int size5, unsigned int size6, bool adjustOnOverflow);
	~ReachabilitySpaceData();

	//! Return the amount of data in bytes
	int getSize();

	inline unsigned int getPos(unsigned int x0, unsigned int x1, unsigned int x2,
								unsigned int x3, unsigned int x4, unsigned int x5)
	{
		return x0 * sizeX0 + x1 * sizeX1 + x2 * sizeX2 + x3 * sizeX3 + x4 * sizeX4 + x5;
	}

	inline unsigned int getPos( unsigned int x[6] )
	{
		return x[0] * sizeX0 + x[1] * sizeX1 + x[2] * sizeX2 + x[3] * sizeX3 + x[4] * sizeX4 + x[5];
	}

	inline void setDatum(	unsigned int x0, unsigned int x1, unsigned int x2,
		unsigned int x3, unsigned int x4, unsigned int x5, unsigned char value)
	{
		unsigned int pos = getPos(x0,x1,x2,x3,x4,x5);
		if (data[pos]==0)
			voxelFilledCount++;
		data[pos] = value;
	}

	inline void setDatum(unsigned int x[6], unsigned char value)
	{
		unsigned int pos = getPos(x);
		if (data[pos]==0)
			voxelFilledCount++;
		data[pos] = value;
	}

	inline void increaseDatum(	unsigned int x0, unsigned int x1, unsigned int x2,
								unsigned int x3, unsigned int x4, unsigned int x5)
	{
		unsigned int pos = getPos(x0,x1,x2,x3,x4,x5);
		unsigned char e = data[pos];
		if (e==0)
			voxelFilledCount++;
		if (e<UCHAR_MAX)
		{
			data[pos]++;
			if (e >= maxEntry)
				maxEntry = e+1;
		} else if (adjustOnOverflow)
			bisectData();
	}
	inline void increaseDatum(	unsigned int x[6] )
	{
		unsigned int pos = getPos(x);
		unsigned char e = data[pos];
		if (e==0)
			voxelFilledCount++;
		if (e<UCHAR_MAX)
		{
			data[pos]++;
			if (e >= maxEntry)
				maxEntry = e+1;
		} else if (adjustOnOverflow)
			bisectData();
	}

	void setData(unsigned char *data);
	const unsigned char *getData() const;

	//! Simulates a multi-dimensional array access
	inline unsigned char get(unsigned int x0, unsigned int x1, unsigned int x2,
		unsigned int x3, unsigned int x4, unsigned int x5) const
	{
		return data[x0 * sizeX0 + x1 * sizeX1 + x2 * sizeX2 + x3 * sizeX3 + x4 * sizeX4 + x5];
	}

	//! Simulates a multi-dimensional array access
	inline unsigned char get( unsigned int x[6] ) const
	{
		return data[x[0] * sizeX0 + x[1] * sizeX1 + x[2] * sizeX2 + x[3] * sizeX3 + x[4] * sizeX4 + x[5]];
	}

	unsigned char getMaxEntry();
	unsigned int getVoxelFilledCount();
	void binarize();

	void bisectData();
private:
	unsigned int sizes[6];
	unsigned int sizeX0,sizeX1,sizeX2,sizeX3,sizeX4;
	unsigned char *data;
	unsigned char maxEntry;
	unsigned int voxelFilledCount;
	bool adjustOnOverflow;
};

/*!
		This class represents an approximation of the reachability distribution of a kinematic chain (e.g. an arm).
		Consists of voxels covering the 6D space for position (XYZ) and orientation (RPY).
		Each voxels holds a counter with the number of successful IK solver calls.
		The discretized reachability space can be written to and loaded from binary files.

		The reachability space is linked to a base coordinate system which is defined by a robot joint.
		This base system is used in order to use the reachability space when the robot is moving.
		I.E. think of an arm of a humanoid where the reachability space is linked to the shoulder.
		When the torso moves, the reachability also changes it's position according to the position of the shoulder.
*/
class VIRTUAL_ROBOT_IMPORT_EXPORT ReachabilitySpace : public boost::enable_shared_from_this<ReachabilitySpace>
{
public:
	friend class CoinVisualizationFactory;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ReachabilitySpace(RobotPtr robot);

	/*!
		Reset all data.
	*/
	void reset();

	/*! 
		Load the reachability data from a binary file.
		Exceptions are thrown on case errors are detected.
	*/
	void load(const std::string &filename);

	/*! 
		Store the reachability data to a binary file.
		Exceptions are thrown on case errors are detected.
	*/
	void save(const std::string &filename);

	/*!
		Return corresponding entry of reachability data 
	*/
	unsigned char getEntry(const Eigen::Matrix4f &globalPose);

	bool isReachable(const Eigen::Matrix4f &globalPose);

	GraspSetPtr getReachableGrasps(GraspSetPtr grasps, ManipulationObjectPtr object);

	int getMaxEntry();
	float getVoxelSize(int dim);
	RobotNodePtr getBaseNode();
	RobotNodePtr getTCP();
	RobotNodeSetPtr getNodeSet();

	/*!
		Initialize and reset all data.
		\param nodeSet The robot node set that should be considered for reachability analysis.
		\param discretizeStepTranslation The extend of a voxel dimension in translational dimensions (x,y,z)
		\param discretizeStepRotation The extend of a voxel dimension in rotational dimensions (roll, pitch, yaw)
		\param minBounds The minimum workspace poses (x,y,z,ro,pi,ya) given in baseNode's coordinate system
		\param maxBounds The maximum workspace poses (x,y,z,ro,pi,ya) given in baseNode's coordinate system
		\param staticCollisionModel The static collision model of the robot. This model does not move when changing the configuration of the RobotNodeSet. If not set no collisions will be checked when building the reachability data
		\param dynamicCollisionModel The dynamic collision model of the robot. This model does move when changing the configuration of the RobotNodeSet. If not set no collisions will be checked when building the reachability data.
		\param baseNode Perform the computations in the coordinate system of this node. If not set, the global pose is used (be careful, when the robot moves around global poses may not be meaningful!)
		\param tcpNode If given, the pose of this node is used for reachability calculations. If not given, the TCP node of the nodeSet is used.
		\param adjustOnOverflow If set the 8bit data is divided by 2 when one voxel entry exceeds 255. Otherwise the entries remain at 255.
	*/
	void initialize(RobotNodeSetPtr nodeSet,
					float discretizeStepTranslation, 
					float discretizeStepRotation, 
					float minBounds[6], 
					float maxBounds[6],
					SceneObjectSetPtr staticCollisionModel = SceneObjectSetPtr(),
					SceneObjectSetPtr dynamicCollisionModel = SceneObjectSetPtr(),
					RobotNodePtr baseNode = RobotNodePtr(), 
					RobotNodePtr tcpNode = RobotNodePtr(),
					bool adjustOnOverflow = true);

	/*!
		Append current TCP pose to Reachability Data
	*/
	void addCurrentTCPPose();

	/*!
		Append a number of random TCP poses to Reachability Data
		\param loops Number of poses that should be appended
		\param checkForSelfCollisions Build a collision-free configuration. If true, random configs are generated until one is collision-free.
	*/
	void addRandomTCPPoses(unsigned int loops, bool checkForSelfCollisions = true);

	/*!
		Generate a random configuration for the robot node set. This configuration is within the joint limits of the current robot node set.
		\param checkForSelfCollisions Build a collision-free configuration. If true, random configs are generated until one is collision-free.
	*/
	bool setRobotNodesToRandomConfig(bool checkForSelfCollisions = true);

	/*!
		Cut all data >1 to 1. This reduces the file size when saving compressed data.
	*/
	void binarize();

	/*!
		Checks for all voxels with entiry==0 if there are neighbors with entries>0.0 If so the entry is set to the averaged value of the neighbors
		\return The number of changed voxels.
	*/
	int fillHoles();

	/*!
	Print status information
	*/
	void print();

	//! returns a random pose that is reachable
	Eigen::Matrix4f sampleReachablePose();

protected:
	//! Specific methods to read/write strings from/to reachability files
	bool readString(std::string &res, std::ifstream &file);
	void writeString(std::ofstream &file, const std::string &value);

	//! General methods to read/write binary data to a file
	template<typename T> T read(std::ifstream &file);
	template<typename T> void readArray(T *res, int num, std::ifstream &file);
	template<typename T> void write(std::ofstream &file, T value);
	template<typename T> void writeArray(std::ofstream &file, const T *value, int num);

	//! (Un-)compress the data
	void uncompressData(const unsigned char *source, int size, unsigned char *dest);
	unsigned char *compressData(const unsigned char *source, int size, int &compressedSize);

	//! Refetch the base joint's transformation, in case this joint has moved
	void updateBaseTransformation();

	int sumAngleReachabilities(int x0, int x1, int x2);

	bool getVoxelFromPose(float x[6], unsigned int v[6]);

	RobotPtr robot;
	RobotNodePtr baseNode;
	RobotNodePtr tcpNode;
	RobotNodeSetPtr nodeSet;
	Eigen::Matrix4f baseTransformation;
	SceneObjectSetPtr staticCollisionModel;
	SceneObjectSetPtr dynamicCollisionModel;

	// Number of processed random configs
	int buildUpLoops;

	// Number of reported collisions
	int collisionConfigs;

	// Tells how to discretize the reachability space
	float discretizeStepTranslation;
	float discretizeStepRotation;
	float minBounds[6];
	float maxBounds[6];

	// Number of voxels in each dimension
	int numVoxels[6];

	// The smallest/greatest tcp workspace pose value reached in each dimension
	float achievedMinValues[6];
	float achievedMaxValues[6];

	// workspace extend in each dimension
	float spaceSize[6];

	ReachabilitySpaceDataPtr data;

	bool adjustOnOverflow;

};


} // namespace VirtualRobot

#endif // _ReachabilitySpace_h_
