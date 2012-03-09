#include "WorkspaceData.h"

#include <fstream>
#include <cmath>
#include <float.h>
#include <limits.h>

namespace VirtualRobot
{

WorkspaceData::WorkspaceData(unsigned int size1, unsigned int size2, unsigned int size3,
                             unsigned int size4, unsigned int size5, unsigned int size6, bool adjustOnOverflow)
{
	unsigned long long size = (unsigned long long)size1 * (unsigned long long)size2 * (unsigned long long)size3 * (unsigned long long)size4 * (unsigned long long)size5 * (unsigned long long)size6;
	if (size>UINT_MAX)
	{
		VR_ERROR << "Could not assign " << size << " bytes of memory (>UINT_MAX). Reduce size of reachability space..." << endl;
	}
	try
	{
	  data = new unsigned char[(unsigned int)size];
	} catch (const std::exception &e)
	{
		VR_ERROR << "Exception: " << e.what() << endl << "Could not assign " << size << " bytes of memory. Reduce size of reachability space..." << endl;
		throw;
	} catch (...)
	{
		VR_ERROR << "Could not assign " << size << " bytes of memory. Reduce size of reachability space..." << endl;
		throw;
	}
		

    sizes[0] = size1;
    sizes[1] = size2;
    sizes[2] = size3;
    sizes[3] = size4;
    sizes[4] = size5;
    sizes[5] = size6;
	sizeX0 = sizes[1]*sizes[2]*sizes[3]*sizes[4]*sizes[5];
	sizeX1 = sizes[2]*sizes[3]*sizes[4]*sizes[5];
	sizeX2 = sizes[3]*sizes[4]*sizes[5];
	sizeX3 = sizes[4]*sizes[5];
	sizeX4 = sizes[5];
	maxEntry = 0;
	voxelFilledCount = 0;
	this->adjustOnOverflow = adjustOnOverflow;

	memset(data,0,getSize()*sizeof(unsigned char));
}

WorkspaceData::~WorkspaceData()
{
	delete[] data;
}

unsigned int WorkspaceData::getSize() const
{
	return sizes[0]*sizes[1]*sizes[2]*sizes[3]*sizes[4]*sizes[5];
}

void WorkspaceData::setData(unsigned char *data)
{
	memcpy(this->data, data, getSize()*sizeof(unsigned char));
}

const unsigned char *WorkspaceData::getData() const
{
	return data;
}

unsigned char WorkspaceData::getMaxEntry() const
{
	return maxEntry;
}

unsigned int WorkspaceData::getVoxelFilledCount() const
{
	return voxelFilledCount;
}

void WorkspaceData::binarize()
{

	for (unsigned int a=0;a<sizes[0];a++)
	{
		for (unsigned int b=0;b<sizes[1];b++)
		{
			for (unsigned int c=0;c<sizes[2];c++)
			{
				for (unsigned int d=0;d<sizes[3];d++)
				{
					for (unsigned int e=0;e<sizes[4];e++)
					{
						for (unsigned int f=0;f<sizes[5];f++)
						{
							unsigned int pos = getPos(a,b,c,d,e,f);
							if (data[pos]>1)
								data[pos] = 1;
						}
					}
				}
			}
		}
	}
	maxEntry = 1;
}

void WorkspaceData::bisectData()
{
	for (unsigned int x0=0;x0<sizes[0];x0++)
		for (unsigned int x1=0;x1<sizes[1];x1++)
			for (unsigned int x2=0;x2<sizes[2];x2++)
				for (unsigned int x3=0;x3<sizes[3];x3++)
					for (unsigned int x4=0;x4<sizes[4];x4++)
						for (unsigned int x5=0;x5<sizes[5];x5++)
						{
							unsigned char c = data[x0 * sizeX0 + x1 * sizeX1 + x2 * sizeX2 + x3 * sizeX3 + x4 * sizeX4 + x5];
							if (c>1)
								data[x0 * sizeX0 + x1 * sizeX1 + x2 * sizeX2 + x3 * sizeX3 + x4 * sizeX4 + x5] = c/2;
						}
	if (maxEntry>1)
	    maxEntry = maxEntry / 2;
}



} // namespace VirtualRobot
