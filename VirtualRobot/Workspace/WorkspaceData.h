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
#ifndef _VirtualRobot_WorkspaceData_h_
#define _VirtualRobot_WorkspaceData_h_

#include "../VirtualRobotImportExport.h"

#include <boost/enable_shared_from_this.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/mpl/assert.hpp>

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace VirtualRobot
{
/*!
	Stores a 6-dimensional array for the vertex data of a workspace representation.
	Internally unsigned char data types are used (0...255)
*/
class VIRTUAL_ROBOT_IMPORT_EXPORT WorkspaceData : public boost::enable_shared_from_this<WorkspaceData>
{
public:	
	/*!
		Constructor, fills the data with 0
	*/
	WorkspaceData(unsigned int size1, unsigned int size2, unsigned int size3,
			      unsigned int size4, unsigned int size5, unsigned int size6, bool adjustOnOverflow);
	~WorkspaceData();

	//! Return the amount of data in bytes
	unsigned int getSize() const;

	inline unsigned int getPos(unsigned int x0, unsigned int x1, unsigned int x2,
							   unsigned int x3, unsigned int x4, unsigned int x5) const
	{
		return x0 * sizeX0 + x1 * sizeX1 + x2 * sizeX2 + x3 * sizeX3 + x4 * sizeX4 + x5;
	}

	inline unsigned int getPos( unsigned int x[6] ) const
	{
		return x[0] * sizeX0 + x[1] * sizeX1 + x[2] * sizeX2 + x[3] * sizeX3 + x[4] * sizeX4 + x[5];
	}

	inline void setDatum(unsigned int x0, unsigned int x1, unsigned int x2,
		                 unsigned int x3, unsigned int x4, unsigned int x5, unsigned char value)
	{
		unsigned int pos = getPos(x0,x1,x2,x3,x4,x5);
		if (data[pos]==0)
			voxelFilledCount++;
		data[pos] = value;
		if (value >= maxEntry)
			maxEntry = value;
	}

	inline void setDatum(unsigned int x[6], unsigned char value)
	{
		unsigned int pos = getPos(x);
		if (data[pos]==0)
			voxelFilledCount++;
		data[pos] = value;
		if (value >= maxEntry)
			maxEntry = value;
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

	unsigned char getMaxEntry() const;
	unsigned int getVoxelFilledCount() const;
	void binarize();

	void bisectData();

	unsigned int sizes[6];
	unsigned int sizeX0,sizeX1,sizeX2,sizeX3,sizeX4;
	unsigned char *data;
	unsigned char maxEntry;
	unsigned int voxelFilledCount;
	bool adjustOnOverflow;
};



} // namespace VirtualRobot

#endif // _WorkspaceData_h_
