#include "WorkspaceRepresentation.h"
#include "../VirtualRobotException.h"
#include "../Robot.h"
#include "../RobotNodeSet.h"
#include "../Compression/CompressionRLE.h"
#include "../Compression/CompressionBZip2.h"
#include "../SceneObjectSet.h"
#include "../Nodes/RobotNode.h"
#include "../Visualization/Visualization.h"
#include "../Visualization/VisualizationFactory.h"
#include "../CollisionDetection/CollisionChecker.h"
#include "../Visualization/ColorMap.h"
#include "../ManipulationObject.h"
#include "../Grasping/Grasp.h"
#include "../Grasping/GraspSet.h"
#include <fstream>
#include <cmath>
#include <float.h>
#include <limits.h>

namespace VirtualRobot
{

WorkspaceRepresentation::WorkspaceRepresentation(RobotPtr robot)
{
	THROW_VR_EXCEPTION_IF(!robot,"Need a robot ptr here");
	this->robot = robot;
	type = "WorkspaceRepresentation";
	versionMajor = 2;
	versionMinor = 5;
	reset();
}

int WorkspaceRepresentation::sumAngleReachabilities(int x0, int x1, int x2) const
{
	int res = 0;
	if (!data->hasEntry(x0,x1,x2))
		return 0;
	for(int d = 0; d < numVoxels[3]; d++)
	{
		for(int e = 0; e < numVoxels[4]; e++)
		{
			for(int f = 0; f < numVoxels[5]; f++)
				res += data->get(x0, x1, x2, d, e, f);
		}
	}
	return res;
}


bool WorkspaceRepresentation::readString(std::string &res, std::ifstream &file)
{
	int length = read<int>(file);
    if(length <= 0)
    {
		VR_WARNING << "Bad string length: " << length << std::endl;
        return false;
    }

    char *data = new char[length+1];
    file.read(data, length);
    data[length] = '\0';
    res = data;
    delete[] data;
    return true;
}

void WorkspaceRepresentation::writeString(std::ofstream &file, const std::string &value)
{
	int len = value.length();
	file.write((char *)&len, sizeof(int));
	file.write(value.c_str(), len);
}

template<typename T> T WorkspaceRepresentation::read(std::ifstream &file)
{
	T t;
	file.read((char *)&t, sizeof(T));
	return t;
}

template<typename T> void WorkspaceRepresentation::readArray(T *res, int num, std::ifstream &file)
{
	file.read((char *)res, num * sizeof(T));
}

template<typename T> void WorkspaceRepresentation::write(std::ofstream &file, T value)
{
	file.write((char *)&value, sizeof(T));
}

template<typename T> void WorkspaceRepresentation::writeArray(std::ofstream &file, const T *value, int num)
{
	file.write((char *)value, num * sizeof(T));
}


void WorkspaceRepresentation::uncompressData(const unsigned char *source, int size, unsigned char *dest)
{
	unsigned char count;
	unsigned char value;

	for(int i = 0; i < size/2; i++)
	{
		count = *source;
		source++;
		value = *source;
		source++;
		memset(dest, (int)value, sizeof(unsigned char) * count);
		dest += count;
	}
}

unsigned char *WorkspaceRepresentation::compressData(const unsigned char *source, int size, int &compressedSize)
{
	// on large arrays sometimes an out-of-memory exception is thrown, so in order to reduce the size of the array, we assume we can compress it
	// hence, we have to check if the compressed size does not exceed the original size on every pos increase
	unsigned char *dest;
	try
	{
		dest  = new unsigned char[/*2 * */size];
	}
	catch (std::exception e)
	{
		VR_ERROR << "Error:" << e.what() << endl << "Could not assign " << size << " bytes of memory. Reduce size of WorkspaceRepresentation data..." << endl;
		throw;
	} catch (...)
	{
		VR_ERROR << "Could not assign " << size << " bytes of memory. Reduce size of WorkspaceRepresentation data..." << endl;
		throw;
	}
	int pos = 0;

	unsigned char curValue = source[0];
	unsigned char count = 1;

	for(int i = 1; i < size; i++)
	{
		if(source[i] == curValue)
		{
			if(count == 255)
			{
				dest[pos] = 255;
				dest[pos+1] = curValue;
				pos += 2;
				THROW_VR_EXCEPTION_IF(pos>=size, "Could not perform run-length compression. Data is too cluttered!!!");

				count = 1;
			}
			else
				count++;
		}
		else
		{
			dest[pos] = count;
			dest[pos+1] = curValue;
			pos += 2;
			THROW_VR_EXCEPTION_IF(pos>=size, "Could not perform run-length compression. Data is too cluttered!!!");

			curValue = source[i];
			count = 1;
		}
	}
	if(count > 0)
	{
		dest[pos] = count;
		dest[pos+1] = curValue;
		pos += 2;
		THROW_VR_EXCEPTION_IF(pos>=size, "Could not perform run-length compression. Data is too cluttered!!!");
	}
	compressedSize = pos;
	return dest;
}

void WorkspaceRepresentation::load(const std::string &filename)
{
	std::ifstream file(filename.c_str(), std::ios::in | std::ios::binary);
	THROW_VR_EXCEPTION_IF(!file, "File could not be read.");
	reset();
	try
	{
		std::string tmpString;
		    
		std::string tmpStr2 = type;
		tmpStr2 += " Binary File";

		// Check file type
		readString(tmpString, file);
		bool fileTypeOK = false;
		if (tmpString == "WorkspaceRepresentation Binary File" ||
		    tmpString == "Reachability Binary File" ||
		    tmpString == "Manipulability Binary File" ||
		    tmpString == "ReachabilitySpace Binary File" ||
		    tmpString == tmpStr2)
		    fileTypeOK = true;
		    
		    
		THROW_VR_EXCEPTION_IF(!fileTypeOK, "Wrong file format.");

		// Check version
		int version[2];
		readArray<int>(version, 2, file);
		// first check if the current version is used
		if (version[0] != versionMajor || version[1] != versionMinor)
		{
			// now check if an older version is used
			THROW_VR_EXCEPTION_IF(
				(version[0] > 2) || 
				(version[0] == 2 && !(version[1] == 0 || version[1] == 1 || version[1] == 2 || version[1] == 3 || version[1] == 4 || version[1] == 5)) || 
				(version[0] == 1 && !(version[1] == 0 || version[1] == 2 || version[1] == 3)
				),	"Wrong file format version");
		}
		//versionMajor = version[0];
		//versionMinor = version[1];
		// Check Robot name
		readString(tmpString, file);
		THROW_VR_EXCEPTION_IF(tmpString != robot->getType(), "Wrong Robot");

		// Check Node Set
		readString(tmpString, file);
		nodeSet = robot->getRobotNodeSet(tmpString);
		THROW_VR_EXCEPTION_IF(!nodeSet, "Node Set does not exist.");
		if(version[0] > 1 || (version[0] == 1 && version[1] > 0) )
		{
			THROW_VR_EXCEPTION_IF(nodeSet->getSize() != read<int>(file), "Node Sets don't match (size differs).");

			// Check joint limits
			std::vector<RobotNodePtr> nodes = nodeSet->getAllRobotNodes();
			for(std::vector<RobotNodePtr>::iterator n = nodes.begin(); n != nodes.end(); n++)
			{
				float limits[2];
				readArray<float>(limits, 2, file);
				if(fabs((*n)->getJointLimitLo() - limits[0]) > 0.01 || fabs((*n)->getJointLimitHi() - limits[1]) > 0.01)
					VR_WARNING << "Joint limit mismatch for " << (*n)->getName() << ", min: " << (*n)->getJointLimitLo() << " / " << limits[0] << ", max: " << (*n)->getJointLimitHi() << " / " << limits[1] << std::endl;
			}
		}

		// Check TCP
		readString(tmpString, file);
		tcpNode = robot->getRobotNode(tmpString);
		THROW_VR_EXCEPTION_IF(!tcpNode, "Unknown TCP");

		// Check Base Joint
		if(version[0] > 1 || (version[0] == 1 &&  version[1] > 0))
		{
			readString(tmpString, file);
			baseNode = robot->getRobotNode(tmpString);
			THROW_VR_EXCEPTION_IF(!baseNode, "Unknown Base Joint");
			//updateBaseTransformation();
		}
		//else
			//baseTransformation.setIdentity();

		// Static collision model
		readString(tmpString, file);
		if(tmpString != "" && tmpString != "not set")
			staticCollisionModel = robot->getRobotNodeSet(tmpString);

		// Dynamic collision model
		readString(tmpString, file);
		if(tmpString != "" && tmpString != "not set")
			dynamicCollisionModel = robot->getRobotNodeSet(tmpString);

		buildUpLoops = read<int>(file);
		collisionConfigs = read<int>(file);
		discretizeStepTranslation = read<float>(file);
		discretizeStepRotation = read<float>(file);
		readArray<int>(numVoxels, 6, file);
		int voxelFilledCount = read<int>(file);
		int maxEntry = read<int>(file);

		for(int i = 0; i < 6; i++)
		{
			minBounds[i] = read<float>(file);
			maxBounds[i] = read<float>(file);
			spaceSize[i] = maxBounds[i] - minBounds[i];
		}

		for(int i = 0; i < 6; i++)
		{
			achievedMinValues[i] = read<float>(file);
			achievedMaxValues[i] = read<float>(file);
		}
		if ((version[0]>2) || (version[0] == 2 && version[1] >= 2))
		{
			if (!customLoad(file))
			{
				VR_ERROR << "Custom loading failed?!" << endl;
			}
		}

		// Read Data
		readString(tmpString, file);
		THROW_VR_EXCEPTION_IF(tmpString != "DATA_START", "Bad file format, expecting DATA_START.");

		int size = numVoxels[0]*numVoxels[1]*numVoxels[2]*numVoxels[3]*numVoxels[4]*numVoxels[5];
		data.reset(new WorkspaceData(numVoxels[0], numVoxels[1], numVoxels[2], numVoxels[3], numVoxels[4], numVoxels[5],true));

		if (version[0]<=1 || (version[0]==2 && version[1]<=3))
		{
			// one data block
			unsigned char *d = new unsigned char[size];	
			if(version[0] == 1 && version[1] <= 2)
			{
				// Data is uncompressed
				readArray<unsigned char>(d, size, file);
			}
			else
			{
				// Data is compressed
				int compressedSize = read<int>(file);
				unsigned char *compressedData = new unsigned char[compressedSize];
				readArray<unsigned char>(compressedData, compressedSize, file);
				if ( (version[0] > 2) ||  (version[0] == 2 && version[1] >= 1))
					CompressionRLE::RLE_Uncompress(compressedData,d,compressedSize);
				else
					uncompressData(compressedData, compressedSize, d);
				delete[] compressedData;
			}
			// convert old data format
			unsigned char *dRot;
			unsigned int sizeX0 = numVoxels[1]*numVoxels[2]*numVoxels[3]*numVoxels[4]*numVoxels[5];
			unsigned int sizeX1 = numVoxels[2]*numVoxels[3]*numVoxels[4]*numVoxels[5];
			unsigned int sizeX2 = numVoxels[3]*numVoxels[4]*numVoxels[5];
			unsigned int sizeX3 = numVoxels[4]*numVoxels[5];
			unsigned int sizeX4 = numVoxels[5];
			dRot = new unsigned char[numVoxels[3]*numVoxels[4]*numVoxels[5]];
			for (int x=0;x<numVoxels[0];x++)
				for (int y=0;y<numVoxels[1];y++)
					for (int z=0;z<numVoxels[2];z++)
					{
						for (int a=0;a<numVoxels[3];a++)
							for (int b=0;b<numVoxels[4];b++)
								for (int c=0;c<numVoxels[5];c++)
								{
									dRot[a*sizeX3 + b*sizeX4 + c] = 
										d[x * sizeX0 + y * sizeX1 + z * sizeX2 + a * sizeX3 + b * sizeX4 + c];
								}

						data->setDataRot(dRot,x,y,z);
					}
			delete [] dRot;
			delete[] d;
		} else
		{
			// data is split, only rotations are given in blocks
			// Data is compressed
			
			bool compressionBZIP2 = false;
			if (version[0]>2 || (version[0]==2 && version[1]>=5))
				compressionBZIP2 = true;
			if (compressionBZIP2)
			{
				int dataSize = numVoxels[3]*numVoxels[4]*numVoxels[5];
				unsigned char *uncompressedData = new unsigned char[dataSize];
				CompressionBZip2Ptr bzip2(new CompressionBZip2(&file));
				for (int x=0;x<numVoxels[0];x++)
					for (int y=0;y<numVoxels[1];y++)
						for (int z=0;z<numVoxels[2];z++)
						{
							int n;
							bzip2->read((void*)(uncompressedData),dataSize,n);
							if (n!=dataSize)
							{
								VR_ERROR << "Invalid number of bytes?!" << endl;
								bzip2->close();
								file.close();
								return;
							}
							data->setDataRot(uncompressedData,x,y,z);
						}
						delete[] uncompressedData;
						bzip2->close();
			} else
			{
				int maxCompressedSize = numVoxels[3]*numVoxels[4]*numVoxels[5]*3;
				unsigned char *compressedData = new unsigned char[maxCompressedSize];
				unsigned char *uncompressedData = new unsigned char[numVoxels[3]*numVoxels[4]*numVoxels[5]];
				for (int x=0;x<numVoxels[0];x++)
					for (int y=0;y<numVoxels[1];y++)
						for (int z=0;z<numVoxels[2];z++)
						{
							int compressedSize = read<int>(file);
						
							readArray<unsigned char>(compressedData, compressedSize, file);
						
							CompressionRLE::RLE_Uncompress(compressedData,uncompressedData,compressedSize);
							data->setDataRot(uncompressedData,x,y,z);
						}
				delete[] compressedData;
				delete[] uncompressedData;
			}
		}


		data->setVoxelFilledCount(voxelFilledCount);
		data->setMaxEntry(maxEntry);

		readString(tmpString, file);
		THROW_VR_EXCEPTION_IF(tmpString != "DATA_END", "Bad file format, expecting DATA_END");
	}
	catch(VirtualRobotException &e)
	{
		VR_ERROR << e.what() << endl;
		file.close();
		throw;
	}
	file.close();
}

void WorkspaceRepresentation::save(const std::string &filename)
{
	THROW_VR_EXCEPTION_IF(!data || !nodeSet, "No WorkspaceRepresentation data loaded");

	std::ofstream file;
	file.open(filename.c_str(), std::ios::out | std::ios::binary);
	THROW_VR_EXCEPTION_IF(!file.is_open(), "Could not open file");

	try
	{
		// File type
		std::string tmpStr = type;
		tmpStr += " Binary File";
		writeString(file, tmpStr);

		// Version
		write<int>(file, versionMajor);
		write<int>(file, versionMinor);

		// Robot type
		writeString(file, robot->getType());

		// Node set name
		writeString(file, nodeSet->getName());

		// Joint limits
		const std::vector<RobotNodePtr> nodes = nodeSet->getAllRobotNodes();
		write<int>(file, nodes.size());
		for(std::vector<RobotNodePtr>::const_iterator n = nodes.begin(); n != nodes.end(); n++)
		{
			write<float>(file, (*n)->getJointLimitLo());
			write<float>(file, (*n)->getJointLimitHi());
		}

		// TCP name
		writeString(file, tcpNode->getName());

		// Base Joint name
		writeString(file, baseNode->getName());

		// Collision models
		if(staticCollisionModel)
			writeString(file, staticCollisionModel->getName());
		else
			writeString(file, "not set");
		if(dynamicCollisionModel)
			writeString(file, dynamicCollisionModel->getName());
		else
			writeString(file, "not set");

		// Build loops
		write<int>(file, buildUpLoops);

		// Collisions
		write<int>(file, collisionConfigs);

		// DiscretizeStep*
		write<float>(file, discretizeStepTranslation);
		write<float>(file, discretizeStepRotation);

		// Number of voxels
		writeArray<int>(file, numVoxels, 6);
		write<int>(file, data->getVoxelFilledCount());
		write<int>(file, data->getMaxEntry());

		// Workspace extend
		for(int i = 0; i < 6; i++)
		{
			write<float>(file, minBounds[i]);
			write<float>(file, maxBounds[i]);
		}

		// Workspace achieved values
		for(int i = 0; i < 6; i++)
		{
			write<float>(file, achievedMinValues[i]);
			write<float>(file, achievedMaxValues[i]);
		}

		if (!customSave(file))
		{
			VR_ERROR << "Custom saving failed?!" << endl;
		}


		// Data
		writeString(file, "DATA_START");
		int size = 0;
		int maxCompressedSize = numVoxels[3]*numVoxels[4]*numVoxels[5]*3;
		/*unsigned char *compressedData = new unsigned char[maxCompressedSize];
		for (int x=0;x<numVoxels[0];x++)
			for (int y=0;y<numVoxels[1];y++)
				for (int z=0;z<numVoxels[2];z++)
				{
					size = CompressionRLE::RLE_Compress(data->getDataRot(x,y,z),compressedData,data->getSizeRot());
					write<int>(file, size);
					if(size > 0)
						writeArray<unsigned char>(file, compressedData, size);
				}
		delete []compressedData;*/
		CompressionBZip2Ptr bzip2(new CompressionBZip2(&file));
		for (int x=0;x<numVoxels[0];x++)
			for (int y=0;y<numVoxels[1];y++)
				for (int z=0;z<numVoxels[2];z++)
				{
					if (!bzip2->write((void*)(data->getDataRot(x,y,z)),data->getSizeRot()))
					{
						VR_ERROR << "Error writing to file.." << endl;
						bzip2->close();
						file.close();
						return;
					}
				}

		bzip2->close();
		writeString(file, "DATA_END");
	}
	catch(VirtualRobotException &e)
	{
		cout << "exception: " << e.what() << endl;
		file.close();
		throw;
	}
	file.close();
}

int WorkspaceRepresentation::getMaxEntry() const
{
	if (!data)
		return 0;
	return data->getMaxEntry();
}

int WorkspaceRepresentation::getMaxEntry( const Eigen::Vector3f &position_global ) const
{
	Eigen::Matrix4f gp;
	gp.setIdentity();
	gp.block(0,3,3,1) = position_global;

	// get voxels
	unsigned int v[6];

	if (!getVoxelFromPose(gp,v))
		return 0;
	return getMaxEntry(v[0],v[1],v[2]);
}

int WorkspaceRepresentation::getMaxEntry( int x0, int x1, int x2 ) const
{
	int maxValue = 0;
	for(int a = 0; a < getNumVoxels(3); a+=1)
	{
		for(int b = 0; b < getNumVoxels(4); b+=1)
		{
			for(int c = 0; c < getNumVoxels(5); c+=1)
			{
				int value = data->get(x0,x1,x2,a, b, c);
				if (value>=maxValue)
					maxValue = value;
			}
		}
	}
	return maxValue;

}

float WorkspaceRepresentation::getVoxelSize(int dim) const
{
	if(dim < 0 || dim > 6)
		return 0.0f;
	if(numVoxels[dim] <= 0)
		return 0.0f;
	return spaceSize[dim] / numVoxels[dim];
}

RobotNodePtr WorkspaceRepresentation::getBaseNode()
{
	return baseNode;
}

RobotNodePtr WorkspaceRepresentation::getTCP()
{
	return tcpNode;
}

RobotNodeSetPtr WorkspaceRepresentation::getNodeSet()
{
	return nodeSet;
}

Eigen::Matrix4f WorkspaceRepresentation::getToLocalTransformation() const
{
	if (baseNode)
		return baseNode->getGlobalPose().inverse();
	else
		return Eigen::Matrix4f::Identity();
}

Eigen::Matrix4f WorkspaceRepresentation::getToGlobalTransformation() const
{
	if (baseNode)
		return baseNode->getGlobalPose();
	else
		return Eigen::Matrix4f::Identity();
}

void WorkspaceRepresentation::toLocal(Eigen::Matrix4f &p) const
{
	p = getToLocalTransformation() * p;
	//if (baseNode)
	//p = baseNode->toLocalCoordinateSystem(p);
}
void WorkspaceRepresentation::toGlobal(Eigen::Matrix4f &p) const
{
	p = getToGlobalTransformation() * p;
	//if (baseNode)
	//p = baseNode->toGlobalCoordinateSystem(p);
}

void WorkspaceRepresentation::toLocalVec(Eigen::Vector3f &positionGlobal) const
{
	Eigen::Matrix4f t;
	t.setIdentity();
	t.block(0,3,3,1)=positionGlobal;
	toLocal(t);
	positionGlobal = t.block(0,3,3,1);
}


void WorkspaceRepresentation::toGlobalVec(Eigen::Vector3f &positionLocal) const
{
	Eigen::Matrix4f t;
	t.setIdentity();
	t.block(0,3,3,1)=positionLocal;
	toGlobal(t);
	positionLocal = t.block(0,3,3,1);
}

void WorkspaceRepresentation::setCurrentTCPPoseEntryIfLower(unsigned char e)
{
	THROW_VR_EXCEPTION_IF(!data || !nodeSet || !tcpNode, "No WorkspaceRepresentation data loaded");

	Eigen::Matrix4f p = tcpNode->getGlobalPose();
	toLocal(p);

	float x[6];
	MathTools::eigen4f2rpy(p,x);

	// check for achieved values
	for (int i=0;i<6;i++)
	{
		if (x[i] < achievedMinValues[i])
			achievedMinValues[i] = x[i];
		if (x[i] > achievedMaxValues[i])
			achievedMaxValues[i] = x[i];
	}

	// get voxels
	unsigned int v[6];
	if (getVoxelFromPose(x,v))
	{
		if (data->get(v) < e)
		{
			data->setDatum(v,e);
		}
	}

	buildUpLoops++;
}

bool WorkspaceRepresentation::getVoxelFromPose(float x[6], unsigned int v[6]) const
{
	int a;
	for (int i=0;i<6;i++)
	{
		a = (int)(((x[i]-minBounds[i]) / spaceSize[i]) * (float)numVoxels[i]);
		if (a<0)
			return false; //pos[i] = 0; // if pose is outside of voxel space, ignore it
		else if (a>=numVoxels[i])
			return false; //pos[i] = m_nVoxels[i]-1; // if pose is outside of voxel space, ignore it
		v[i] = a;
	}
	return true;
}

bool WorkspaceRepresentation::getVoxelFromPose( const Eigen::Matrix4f &globalPose, unsigned int v[6] ) const
{
	float x[6];

	Eigen::Matrix4f p = globalPose;
	toLocal(p);
	//if (baseNode)
		//p = baseNode->toLocalCoordinateSystem(p);

	MathTools::eigen4f2rpy(p,x);
	return getVoxelFromPose(x,v);
}

bool WorkspaceRepresentation::setRobotNodesToRandomConfig( VirtualRobot::RobotNodeSetPtr nodeSet, bool checkForSelfCollisions /*= true*/ )
{
	static const float randMult = (float)(1.0/(double)(RAND_MAX));
	float rndValue;
	float minJ, maxJ;
	Eigen::VectorXf v(nodeSet->getSize());
	float maxLoops = 1000;

	int loop = 0;
	do 
	{
		for (unsigned int i=0;i<nodeSet->getSize();i++)
		{
			rndValue = (float)rand() * randMult; // value from 0 to 1
			minJ = (*nodeSet)[i]->getJointLimitLo();
			maxJ = (*nodeSet)[i]->getJointLimitHi();
			v[i] = minJ + ((maxJ - minJ)*rndValue);
		}
		robot->setJointValues(nodeSet,v);
		// check for collisions
		if (!checkForSelfCollisions || !staticCollisionModel || !dynamicCollisionModel)
			return true;
		if (!robot->getCollisionChecker()->checkCollision(staticCollisionModel,dynamicCollisionModel))
			return true;
		collisionConfigs++;
		loop++;
	} while (loop<maxLoops);
	return false;
}



void WorkspaceRepresentation::addPose(const Eigen::Matrix4f &globalPose)
{
	VR_ASSERT(data);
	Eigen::Matrix4f p = globalPose;
	toLocal(p);

	float x[6];
	MathTools::eigen4f2rpy(p,x);

	// check for achieved values
	for (int i=0;i<6;i++)
	{
		if (x[i] < achievedMinValues[i])
			achievedMinValues[i] = x[i];
		if (x[i] > achievedMaxValues[i])
			achievedMaxValues[i] = x[i];
	}

	// get voxels
	unsigned int v[6];
	if (getVoxelFromPose(x,v))
	{
		data->increaseDatum(v);
	}

	buildUpLoops++;
}


void WorkspaceRepresentation::print()
{
	cout << "-----------------------------------------------------------" << endl;
	cout << type << " - Status:" << endl;
	
	if (data)
	{
		if (nodeSet)
			cout << "Kinematic Chain / RobotNodeSet: " << nodeSet->getName() << endl;
		cout << "Base Joint: ";
		if (baseNode)
			cout << baseNode->getName() << endl;
		else 
			cout << "<GLOBAL POSE>" << endl;
		cout << "TCP Joint: ";
		if (tcpNode)
			cout << tcpNode->getName() << endl;
		else 
			cout << "<not set>" << endl;
		cout << "CollisionModel static: ";
		if (staticCollisionModel)
			cout << staticCollisionModel->getName() << endl;
		else 
			cout << "<not set>" << endl;
		cout << "CollisionModel dynamic: ";
		if (dynamicCollisionModel)
			cout << dynamicCollisionModel->getName() << endl;
		else 
			cout << "<not set>" << endl;
		
		cout << "Used " << buildUpLoops << " loops for building the random configs " << endl;
		cout << "Discretization step sizes: Translation: " << discretizeStepTranslation << " - Rotation: " << discretizeStepRotation << endl;
		cout << type << " data extends: " << numVoxels[0] << "x" << numVoxels[1] << "x" << numVoxels[2] << "x" << numVoxels[3] << "x" << numVoxels[4] << "x" << numVoxels[5] << endl;
		cout << "Filled " << data->getVoxelFilledCount() << " of " << (numVoxels[0]*numVoxels[1]*numVoxels[2]*numVoxels[3]*numVoxels[4]*numVoxels[5]) << " voxels" << endl;
		cout << "Collisions: " << collisionConfigs << endl;
		cout << "Maximum entry in a voxel: " << (int)data->getMaxEntry() << endl;
		cout << type << " workspace extend (as defined on construction):" << endl;
		cout << "Min boundary (local): ";
		for (int i=0;i<6;i++)
			cout << minBounds[i] << ",";
		cout << endl;
		cout << "Max boundary (local): ";
		for (int i=0;i<6;i++)
			cout << maxBounds[i] << ",";
		cout << endl;
		cout << "6D values achieved during buildup:" << endl;
		cout << "Minimum 6D values: ";
		for (int i=0;i<6;i++)
			cout << achievedMinValues[i] << ",";
		cout << endl;
		cout << "Maximum 6D values: ";
		for (int i=0;i<6;i++)
			cout << achievedMaxValues[i] << ",";
		cout << endl;
		customPrint();
	} else
	{
		cout << type << " not created yet..." << endl;
	}
	cout << "-----------------------------------------------------------" << endl;
	cout << endl;
}

void WorkspaceRepresentation::reset()
{
	data.reset();
	nodeSet.reset();
	tcpNode.reset();
	baseNode.reset();
	staticCollisionModel.reset();
	dynamicCollisionModel.reset();
	buildUpLoops = 0;
	collisionConfigs = 0;
	discretizeStepTranslation = 0;
	discretizeStepRotation = 0;
	for (int i=0;i<6;i++)
	{
		minBounds[i] = FLT_MAX;
		maxBounds[i] = -FLT_MAX;
		achievedMinValues[i] = FLT_MAX;
		achievedMaxValues[i] = -FLT_MAX;
		numVoxels[i] = 0;
		spaceSize[i] = 0;
	}
	//baseTransformation.setIdentity();
}

void WorkspaceRepresentation::initialize( RobotNodeSetPtr nodeSet, float discretizeStepTranslation, float discretizeStepRotation, 
									float minBounds[6], float maxBounds[6],  
									SceneObjectSetPtr staticCollisionModel,
									SceneObjectSetPtr dynamicCollisionModel,
									RobotNodePtr baseNode /*= RobotNodePtr()*/, 
									RobotNodePtr tcpNode /*= RobotNodePtr()*/,
									bool adjustOnOverflow /* = true */)
{
	reset();
	THROW_VR_EXCEPTION_IF( (discretizeStepTranslation<=0.0f || discretizeStepRotation<=0.0f), "Need positive discretize steps");
	for (int i=0;i<6;i++)
	{
		THROW_VR_EXCEPTION_IF(minBounds[i] >= maxBounds[i], "Min/MaxBound error");
	}
	THROW_VR_EXCEPTION_IF(!nodeSet,"NULL data, need a nodeSet");
	THROW_VR_EXCEPTION_IF(!nodeSet->isKinematicChain(),"nodeSet must be a valid kinematic chain!");
	this->nodeSet = nodeSet;
	this->tcpNode = nodeSet->getTCP();
	if (tcpNode)
		this->tcpNode = tcpNode;
	THROW_VR_EXCEPTION_IF(!robot->hasRobotNode(this->tcpNode),"robot does not know tcp:" << this->tcpNode->getName());
	this->baseNode = baseNode;
	if (baseNode && !robot->hasRobotNode(baseNode))
	{
		THROW_VR_EXCEPTION("Robot does not know basenode:" << baseNode->getName());
	}
	THROW_VR_EXCEPTION_IF (nodeSet->hasRobotNode(baseNode)," baseNode is part of RobotNodeSet! This is not a good idea, since the globalPose of the baseNode will change during buildup of WorkspaceRepresentation data...");
	this->staticCollisionModel = staticCollisionModel;
	this->dynamicCollisionModel = dynamicCollisionModel;
	if (!staticCollisionModel || !dynamicCollisionModel)
	{
		staticCollisionModel.reset();
		dynamicCollisionModel.reset();
	} else
	{
		THROW_VR_EXCEPTION_IF(staticCollisionModel->getCollisionChecker() != dynamicCollisionModel->getCollisionChecker(),"Need same collision checker instance!");
	}

	// build data
	this->discretizeStepTranslation = discretizeStepTranslation;
	this->discretizeStepRotation = discretizeStepRotation;
	for (int i=0; i<6; i++)
	{
		this->minBounds[i] = minBounds[i];
		this->maxBounds[i] = maxBounds[i];
		spaceSize[i] = maxBounds[i] - minBounds[i];

		if (i<3)
			numVoxels[i] = (int)(spaceSize[i] / discretizeStepTranslation) + 1;
		else
			numVoxels[i] = (int)(spaceSize[i] / discretizeStepRotation) + 1;
		THROW_VR_EXCEPTION_IF( (numVoxels[i]<=0), " numVoxels <= 0 in dimension " << i);
	}

	data.reset(new WorkspaceData(numVoxels[0],numVoxels[1],numVoxels[2],numVoxels[3],numVoxels[4],numVoxels[5],adjustOnOverflow));
}

void WorkspaceRepresentation::binarize()
{
	if (data)
		data->binarize();
}

unsigned char WorkspaceRepresentation::getEntry( const Eigen::Matrix4f &globalPose ) const
{
	if (!data)
	{
		VR_ERROR << "NULL DATA" << endl;
		return 0;
	}

	// get voxels
	unsigned int v[6];
	if (getVoxelFromPose(globalPose,v))
	{
		return data->get(v);
	} else
	{
		// position is outside WorkspaceRepresentation data
		return 0;
	}


}


Eigen::Matrix4f WorkspaceRepresentation::getPoseFromVoxel(unsigned int v[6],bool transformToGlobalPose)
{
	float x[6];
	for (int j=0;j<6;j++)
	{
		x[j] = minBounds[j] + ((float)v[j]+0.5f) * getVoxelSize(j);
	}
	Eigen::Matrix4f m;
	MathTools::posrpy2eigen4f(x,m);
	if (transformToGlobalPose)
		toGlobal(m);
	return m;
}

Eigen::Matrix4f WorkspaceRepresentation::sampleCoveredPose()
{
	int maxLoops = 10000;
	int i = 0;
	Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
	unsigned int nV[6];
	float x[6];
	while (i<maxLoops)
	{
		for (int j=0;j<6;j++)
		{
			nV[j] = rand() % numVoxels[j];
		}
		if (data->get(nV)>0)
		{
			// create pose

			for (int j=0;j<6;j++)
			{
				x[j] = minBounds[j] + ((float)nV[j]+0.5f) * getVoxelSize(j);
			}
			MathTools::posrpy2eigen4f(x,m);
			toGlobal(m);
			//if (baseNode)
				//m = baseNode->toGlobalCoordinateSystem(m);
			return m;

		}
	
		i++;
	}

	VR_ERROR << "Could not find a valid pose?!" << endl;
	return m;
}

int WorkspaceRepresentation::fillHoles()
{
	unsigned int x[6];
	int res = 0;
	for (x[0]=1; x[0]<(unsigned int)numVoxels[0]-1; x[0]++)
		for (x[1]=1; x[1]<(unsigned int)numVoxels[1]-1; x[1]++)
			for (x[2]=1; x[2]<(unsigned int)numVoxels[2]-1; x[2]++)
				for (x[3]=1; x[3]<(unsigned int)numVoxels[3]-1; x[3]++)
					for (x[4]=1; x[4]<(unsigned int)numVoxels[4]-1; x[4]++)
						for (x[5]=1; x[5]<(unsigned int)numVoxels[5]-1; x[5]++)
						{
							if (data->get(x)>0)
							{
								int sum = 0;
								int count = 0;
								for (int i=0;i<6;i++)
								{
									x[i]--;
									if (data->get(x)>0)
									{
										sum += data->get(x);
										count++;
									}
									x[i]++;x[i]++;
									if (data->get(x)>0)
									{
										sum += data->get(x);
										count++;
									}
									x[i]--;
								}
								if (count>1)
								{
									res++;
									sum /= count;
									data->setDatum(x,(unsigned char)sum);
								}
								

							}
						}
	return res;
}

int WorkspaceRepresentation::getNumVoxels( int dim ) const
{
	VR_ASSERT((dim>=0 && dim<6));

	return numVoxels[dim];
}

float WorkspaceRepresentation::getMinBound( int dim ) const
{
	VR_ASSERT((dim>=0 && dim<6));

	return minBounds[dim];
}

float WorkspaceRepresentation::getMaxBound( int dim ) const
{
	VR_ASSERT((dim>=0 && dim<6));

	return maxBounds[dim];
}

unsigned char WorkspaceRepresentation::getVoxelEntry(unsigned int a, unsigned int b, unsigned int c, unsigned int d, unsigned int e, unsigned int f) const
{
	if (a<0 || b<0 || c<0 || d<0 || e<0 || f<0 
		|| int(a)>=numVoxels[0] || int(b)>=numVoxels[1] || int(c)>=numVoxels[2] || int(d)>=numVoxels[3] || int(e)>=numVoxels[4] || int(f)>=numVoxels[5])
		return 0;
	return data->get(a,b,c,d,e,f);
}

int WorkspaceRepresentation::getMaxSummedAngleReachablity()
{
	int maxValue = 0;
	for(int a = 0; a < getNumVoxels(0); a+=1)
	{
		for(int b = 0; b < getNumVoxels(1); b+=1)
		{
			for(int c = 0; c < getNumVoxels(2); c+=1)
			{
				int value = sumAngleReachabilities(a, b, c);
				if (value>=maxValue)
					maxValue = value;
			}
		}
	}
	return maxValue;
}

bool WorkspaceRepresentation::isCovered( const Eigen::Matrix4f &globalPose )
{
	return (getEntry(globalPose) > 0);
}


void WorkspaceRepresentation::setVoxelEntry( unsigned int v[6], unsigned char e )
{
	data->setDatum(v,e);
	buildUpLoops++;
}

void WorkspaceRepresentation::setCurrentTCPPoseEntry( unsigned char e )
{
	THROW_VR_EXCEPTION_IF(!data || !nodeSet || !tcpNode, "No WorkspaceRepresentation data loaded");

	Eigen::Matrix4f p = tcpNode->getGlobalPose();
	setEntry(p,e);

}

bool WorkspaceRepresentation::checkForParameters( RobotNodeSetPtr nodeSet, float steps, float storeMinBounds[6], float storeMaxBounds[6],RobotNodePtr baseNode, RobotNodePtr tcpNode )
{
	if (!robot || !nodeSet || !nodeSet->isKinematicChain())
	{
		VR_WARNING << "invalid data" << endl;
		return false;
	}
	if (!tcpNode)
		tcpNode = nodeSet->getTCP();
	if (!robot->hasRobotNode(tcpNode))
	{
		VR_ERROR << "robot does not know tcp:" << tcpNode->getName() << endl;
		return false;
	}
	if (baseNode && !robot->hasRobotNode(baseNode))
	{
		VR_ERROR << "robot does not know baseNode:" << baseNode->getName() << endl;
		return false;
	}

	for (int i=0;i<6;i++)
	{
		storeMinBounds[i] = FLT_MAX;
		storeMaxBounds[i] = -FLT_MAX;
	}
	Eigen::VectorXf c;
	nodeSet->getJointValues(c);
	bool visuSate = robot->getUpdateVisualizationStatus();
	robot->setUpdateVisualization(false);

	for (int i=0;i<steps;i++)
	{
		setRobotNodesToRandomConfig(nodeSet,false);
		Eigen::Matrix4f p = tcpNode->getGlobalPose();
		toLocal(p);
		//if (baseNode)
			//p = baseNode->toLocalCoordinateSystem(p);

		float x[6];
		MathTools::eigen4f2rpy(p,x);

		// check for achieved values
		for (int i=0;i<6;i++)
		{
			if (x[i] < storeMinBounds[i])
				storeMinBounds[i] = x[i];
			if (x[i] > storeMaxBounds[i])
				storeMaxBounds[i] = x[i];
		}
	}
	robot->setJointValues(nodeSet,c);

	robot->setUpdateVisualization(visuSate);

	// assume higher values
	for (int i=0;i<6;i++)
	{
		float sizex = storeMaxBounds[i] - storeMinBounds[i];
		float factor = 0.1f;
		if (i>2)
			factor = 0.05f; // adjustment for rotation is smaller
		storeMinBounds[i] -= sizex * factor;
		storeMaxBounds[i] += sizex * factor;
	}
	return true;

}

WorkspaceRepresentation::WorkspaceCut2DPtr WorkspaceRepresentation::createCut( const Eigen::Matrix4f& referencePose, float cellSize ) const
{
	WorkspaceCut2DPtr result(new WorkspaceCut2D());
	result->referenceGlobalPose = referencePose;

	Eigen::Vector3f minBB,maxBB;

	getWorkspaceExtends(minBB,maxBB);
	result->minBounds[0] = minBB(0);
	result->maxBounds[0] = maxBB(0);
	result->minBounds[1] = minBB(1);
	result->maxBounds[1] = maxBB(1);

	THROW_VR_EXCEPTION_IF(cellSize<=0.0f, "Invalid parameter");

	float sizeX = result->maxBounds[0] - result->minBounds[0];
	int numVoxelsX = (int)(sizeX / cellSize);
	float sizeY = result->maxBounds[1] - result->minBounds[1];
	int numVoxelsY = (int)(sizeY / cellSize);


	Eigen::Matrix4f tmpPose = referencePose;

	result->entries.resize(numVoxelsX,numVoxelsY);


	for (int a=0;a<numVoxelsX;a++)
	{
		tmpPose(0,3) = result->minBounds[0] + (float)a * cellSize + 0.5f*cellSize;
		for (int b=0;b<numVoxelsY;b++)
		{
			tmpPose(1,3) = result->minBounds[1] + (float)b * cellSize + 0.5f*cellSize;
			result->entries(a,b) = getEntry(tmpPose);
		}
	}
	return result;
}

bool WorkspaceRepresentation::getWorkspaceExtends( Eigen::Vector3f &storeMinBBox, Eigen::Vector3f &storeMaxBBox ) const
{
	Eigen::Vector3f quadPos[8];
	float x,y,z;
	for (int i=0;i<8;i++)
	{
		if (i%2 == 0)
			x = minBounds[0];
		else
			x = maxBounds[0];
		if ((i>>1) % 2 == 0)
			y = minBounds[1];
		else
			y = maxBounds[1];
		if ((i>>2) % 2 == 0)
			z = minBounds[2];
		else
			z = maxBounds[2];
		quadPos[i](0) = x;
		quadPos[i](1) = y;
		quadPos[i](2) = z;
		toGlobalVec(quadPos[i]);
	}
	storeMinBBox = quadPos[0];
	storeMaxBBox = quadPos[0];
	for (int k=0;k<8;k++)
	{
		for (int i=0;i<3;i++)
		{
			if (quadPos[k](i) < storeMinBBox(i))
			{
				storeMinBBox(i) = quadPos[k](i);
			} 
			if (quadPos[k](i) > storeMaxBBox(i))
			{
				storeMaxBBox(i) = quadPos[k](i);
			} 
		}
	}
	return true;
}

std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr> WorkspaceRepresentation::createCutTransformations( WorkspaceRepresentation::WorkspaceCut2DPtr cutXY, RobotNodePtr referenceNode )
{	
	THROW_VR_EXCEPTION_IF(!cutXY,"NULL data");
	
	std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr> result;

	//float x,y,z;
	//z = cutXY->referenceGlobalPose(2,3);

	int nX = cutXY->entries.rows();
	int nY = cutXY->entries.cols();

	float sizeX = (cutXY->maxBounds[0] - cutXY->minBounds[0]) / (float)nX;
	float sizeY = (cutXY->maxBounds[1] - cutXY->minBounds[1]) / (float)nY;

	for (int x = 0; x<nX; x++)
	{
		for (int y = 0; y<nY; y++)
		{
			int v = cutXY->entries(x,y);
			if (v>0)
			{
				WorkspaceCut2DTransformationPtr tp(new WorkspaceCut2DTransformation());
				tp->value = v;
				float xPos = cutXY->minBounds[0] + (float)x * sizeX + 0.5f*sizeX; // center of voxel
				float yPos = cutXY->minBounds[1] + (float)y * sizeY + 0.5f*sizeY; // center of voxel
				tp->transformation = cutXY->referenceGlobalPose;
				tp->transformation(0,3) = xPos;
				tp->transformation(1,3) = yPos;
				if (referenceNode)
					tp->transformation = referenceNode->toLocalCoordinateSystem(tp->transformation);
				result.push_back(tp);
			}
		}
	}
	return result;
}

float WorkspaceRepresentation::getDiscretizeParameterTranslation()
{
	return discretizeStepTranslation;
}

float WorkspaceRepresentation::getDiscretizeParameterRotation()
{
	return discretizeStepRotation;
}

void WorkspaceRepresentation::setEntry( const Eigen::Matrix4f &poseGlobal, unsigned char e )
{
    setEntryCheckNeighbors(poseGlobal,e,0);
}

void WorkspaceRepresentation::setEntryCheckNeighbors( const Eigen::Matrix4f &poseGlobal, unsigned char e, unsigned int neighborVoxels )
{
    Eigen::Matrix4f p = poseGlobal;
    toLocal(p);

    float x[6];
    MathTools::eigen4f2rpy(p,x);

    // check for achieved values
    for (int i=0;i<6;i++)
    {
        if (x[i] < achievedMinValues[i])
            achievedMinValues[i] = x[i];
        if (x[i] > achievedMaxValues[i])
            achievedMaxValues[i] = x[i];
    }

    // get voxels
    unsigned int v[6];
    if (getVoxelFromPose(x,v))
    {
#if 0
        cout << "pose:";
        for (int i=0;i<6;i++)
            cout << x[i] << ",";
        cout << "Voxel:";
        for (int i=0;i<6;i++)
            cout << v[i] << ",";
        cout << endl;
#endif
        data->setDatumCheckNeighbors(v,e,neighborVoxels);
    }

    buildUpLoops++;

}

MathTools::OOBB WorkspaceRepresentation::getOOBB(bool achievedValues) const
{
	Eigen::Vector3f minBB;
	Eigen::Vector3f maxBB;
	if (achievedValues)
	{
		minBB << achievedMinValues[0],achievedMinValues[1],achievedMinValues[2];
		maxBB << achievedMaxValues[0],achievedMaxValues[1],achievedMaxValues[2];
	} else
	{
		minBB << minBounds[0],minBounds[1],minBounds[2];
		maxBB << maxBounds[0],maxBounds[1],maxBounds[2];
	}
	MathTools::OOBB oobb(minBB,maxBB,getToGlobalTransformation());
	return oobb;
}

void WorkspaceRepresentation::clear()
{
	data->clear();
	buildUpLoops = 0;
	collisionConfigs = 0;
	for (int i=0;i<6;i++)
	{
		achievedMinValues[i] = FLT_MAX;
		achievedMaxValues[i] = -FLT_MAX;
	}
}

bool WorkspaceRepresentation::hasEntry( unsigned int x, unsigned int y, unsigned int z )
{
	return data->hasEntry(x,y,z);
}

} // namespace VirtualRobot
