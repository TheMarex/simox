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
#ifndef _VirtualRobot_WorkspaceRepresentation_h_
#define _VirtualRobot_WorkspaceRepresentation_h_

#include "../VirtualRobotImportExport.h"
#include "WorkspaceData.h"


#include <boost/enable_shared_from_this.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/mpl/assert.hpp>

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace VirtualRobot
{

/*!
		This class represents a voxelized approximation of the workspace that is covered by a kinematic chain of a robot. 
		The voxel grid covers the 6d Cartesian space: xyz translations (mm) and rpy orientations.
		Each voxels holds a counter (uchar) that holds information, e.g. about reachbaility.
		The discretized data can be written to and loaded from binary files.

		The data is linked to a base coordinate system which is defined by a robot joint.
		This base system is used to align the data when the robot is moving.
		I.E. think of an arm of a humanoid where the workspace representation is linked to the shoulder.
		When the torso moves, the data representation also changes it's position according to the position of the shoulder.
*/

class VIRTUAL_ROBOT_IMPORT_EXPORT WorkspaceRepresentation : public boost::enable_shared_from_this<WorkspaceRepresentation>
{
public:
	friend class CoinVisualizationFactory;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	WorkspaceRepresentation(RobotPtr robot);

	/*!
		Reset all data.
	*/
	virtual void reset();

	/*! 
		Load the reachability data from a binary file.
		Exceptions are thrown on case errors are detected.
	*/
	virtual void load(const std::string &filename);

	/*! 
		Store the reachability data to a binary file.
		Exceptions are thrown on case errors are detected.
	*/
	virtual void save(const std::string &filename);

	/*!
		Return corresponding entry of reachability data 
	*/
	unsigned char getEntry(const Eigen::Matrix4f &globalPose);

    //! Returns the maximum entry of a voxel.
	int getMaxEntry() const;
	
	//! returns the extends of a voxel at corresponding dimension.
	float getVoxelSize(int dim) const;
	
	//! The base node of this reachability data
	RobotNodePtr getBaseNode();
	
	//! The corresponding TCP
	RobotNodePtr getTCP();
	
	//! The kinematic chain that is covered by this reachability data
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
	virtual void initialize(RobotNodeSetPtr nodeSet,
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
		Sets entry that corresponds to TCP pose to e, if current entry is lower than e.
		Therefore the corresponding voxel of the current TCP pose is determined and its entry is adjusted. 
	*/
	virtual void setCurrentTCPPoseEntryIfLower(unsigned char e);
	
	/*!
		Sets entry that corresponds to TCP pose to e.
		Therefore the corresponding voxel of the current TCP pose is determined and its entry is set. 
	*/
	virtual void setCurrentTCPPoseEntry(unsigned char e);


	/*!
		Generate a random configuration for the robot node set. This configuration is within the joint limits of the current robot node set.
		\param checkForSelfCollisions Build a collision-free configuration. If true, random configs are generated until one is collision-free.
	*/
	virtual bool setRobotNodesToRandomConfig(bool checkForSelfCollisions = true);

	/*!
		Cut all data >1 to 1. This reduces the file size when saving compressed data.
	*/
	virtual void binarize();

	/*!
		Checks for all voxels with entiry==0 if there are neighbors with entries>0. 
		If so the entry is set to the averaged value of the neighbors
		\return The number of changed voxels.
	*/
	virtual int fillHoles();

	/*!
	Print status information
	*/
	virtual void print();

	//! returns a random pose that is covered by the workspace data
	Eigen::Matrix4f sampleCoveredPose();
	    
	/*!
		Returns true, if the corresponding voxel entry is nun zero.
	*/
	bool isCovered(const Eigen::Matrix4f &globalPose);

	virtual int getNumVoxels(int dim) const;
	virtual float getMinBound(int dim) const;
	virtual float getMaxBound(int dim) const;

	virtual unsigned char getVoxelEntry(unsigned int a, unsigned int b, unsigned int c, unsigned int d, unsigned int e, unsigned int f) const;

	/*!
		Sums all angle (x3,x4,x5) entries for the given position.
	*/
	virtual int sumAngleReachabilities(int x0, int x1, int x2) const;

	/*!
		Searches all angle entries (x3,x4,x5) for maximum entry.
		(x0,x1,x2) is the voxel position.
	*/
	virtual int getMaxEntry(int x0, int x1, int x2) const;

	/*!
		Returns the maximum reachability entry that can be achieved by an arbitrary orientation at the given position.
	*/
	virtual int getMaxEntry( const Eigen::Vector3f &position_global ) const;

	/*!
		Get the corresponding voxel. 
		If false is returned the pose is outside the covered workspace.
	*/
	virtual bool getVoxelFromPose(const Eigen::Matrix4f &globalPose, unsigned int v[6]) const;

	/*!
		Returns the maximum that can be achieved by calling sumAngleReachabilities() 
	*/
	virtual int getMaxSummedAngleReachablity();


protected:
	//! Specific methods to read/write strings from/to reachability files
	bool readString(std::string &res, std::ifstream &file);
	void writeString(std::ofstream &file, const std::string &value);

	//! General methods to read/write binary data to a file
	template<typename T> T read(std::ifstream &file);
	template<typename T> void readArray(T *res, int num, std::ifstream &file);
	template<typename T> void write(std::ofstream &file, T value);
	template<typename T> void writeArray(std::ofstream &file, const T *value, int num);

	//! Uncompress the data
	void uncompressData(const unsigned char *source, int size, unsigned char *dest);
	//! Compress the data
	unsigned char *compressData(const unsigned char *source, int size, int &compressedSize);

	//! Refetch the base joint's transformation, in case this joint has moved
	void updateBaseTransformation();

	virtual bool getVoxelFromPose(float x[6], unsigned int v[6]) const;


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

	// Tells how to discretize the reachability data
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

	WorkspaceDataPtr data;

	bool adjustOnOverflow;
	
	std::string type;

};


} // namespace VirtualRobot

#endif // _VirtualRobot_WorkspaceRepresentation_h_
