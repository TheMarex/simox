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
* @author     Nikolaus Vahrenkamp
* @copyright  2013 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_FileIO_h_
#define _VirtualRobot_FileIO_h_

#include "../VirtualRobotImportExport.h"

#include <vector>
#include <fstream>

namespace VirtualRobot
{
namespace FileIO
{

	template<typename T> inline T read(std::ifstream &file)
	{
		T t;
		file.read((char *)&t, sizeof(T));
		return t;
	}

	template<typename T> inline void readArray(T *res, int num, std::ifstream &file)
	{
		file.read((char *)res, num * sizeof(T));
	}

	template<typename T> inline void write(std::ofstream &file, T value)
	{
		file.write((char *)&value, sizeof(T));
	}

	template<typename T> inline void writeArray(std::ofstream &file, const T *value, int num)
	{
		file.write((char *)value, num * sizeof(T));
	}

	inline bool readString(std::string &res, std::ifstream &file)
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

	inline void writeString(std::ofstream &file, const std::string &value)
	{
		int len = value.length();
		file.write((char *)&len, sizeof(int));
		file.write(value.c_str(), len);
	}
}

} // namespace VirtualRobot

#endif // _VirtualRobot_FileIO_h_
