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
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_RuntimeEnv_h_
#define _VirtualRobot_RuntimeEnv_h_

#include "VirtualRobotImportExport.h"

#include <string>
#include <vector>
#include <map>
#include <Eigen/Core>

namespace VirtualRobot 
{
/*!
	The runtime environment holds data paths and program arguments.
*/
class VIRTUAL_ROBOT_IMPORT_EXPORT RuntimeEnvironment
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		/*!
			Return vector of all data paths.
		*/
		static std::vector< std::string > getDataPaths();
		
		/*!
			Add a path to global data path vector. These paths are searched within the getDataFileAbsolute() method.
			Only valid paths are processed.
			\param path The path to add.
			\param quiet If set, invalid paths are quietly ignored. Otherwise an error is printed.
		*/
		static void addDataPath(const std::string &path, bool quiet = false);

		/*!
			Enable the command line search for given key. Only keys that are enabled can later be accessed with the getValue() method.
		*/
		static void considerKey( const std::string &key);

		/*!
			Tries to find a file with name fileName. Therefore the working directory followed by all data paths are checked if the file can be found.
			\param fileName Could be a filename or a relative path. In case the file could be located, the absolute path is stored in place.
			\return True when the file could be located (the result will be stored in fileName).
		*/
		static bool getDataFileAbsolute (std::string &fileName);

		/*!
			The command line parameters can be passed in order to generate a map of key/value pairs.
			All "--key value" pairs for which the key was enabled with the allowCommandLineOption() method are processed and stored as std::strings in a std::map.
			Further all "--data-path <path>" entries are extracted and the according data paths are stored.
			All unrecognized options are also stored.
		*/
		static void processCommandLine(int argc, char *argv[]);

		/*!
			Manually add a key/value pair.
		*/
		static void addKeyValuePair(const std::string &key,const std::string &value);

		/*!
			Return the corresponding vale to key. If key cannot be found, an empty string is returned.
		*/
		static std::string getValue(const std::string &key);
		static bool hasValue(const std::string &key);

		//! return all key value pairs
		static std::map< std::string, std::string > getKeyValuePairs();

		/*!
			Converts strings as '(a,b,c)' to 3dim Vectors.
		*/
		static bool toVector3f(const std::string &s, Eigen::Vector3f &storeResult);

		//! Print status 
		static void print();
	protected:

		RuntimeEnvironment(){}
		virtual ~RuntimeEnvironment(){}
		static bool pathInitialized;
		static void init()
		{
			if (!pathInitialized)
			{
				pathInitialized = true;
#ifdef VR_BASE_DIR
				addDataPath(std::string(VR_BASE_DIR),true);
				addDataPath(std::string(VR_BASE_DIR "/data"),true);
#endif
#ifdef SIMOX_BASE_DIR
				addDataPath(std::string(SIMOX_BASE_DIR),true);
				addDataPath(std::string(SIMOX_BASE_DIR "/data"),true);
#endif
#ifdef SABA_BASE_DIR
				addDataPath(std::string(SABA_BASE_DIR),true);
				addDataPath(std::string(SABA_BASE_DIR "/data"),true);
#endif
#ifdef GRASPSTUDIO_BASE_DIR
				addDataPath(std::string(GRASPSTUDIO_BASE_DIR),true);
				addDataPath(std::string(GRASPSTUDIO_BASE_DIR "/data"),true);
#endif
			}
		}


		static std::vector< std::string > processKeys;
		static std::vector< std::string > dataPaths;
		static std::vector< std::string > unrecognizedOptions;

		static std::map< std::string, std::string > keyValues;
};

} // namespace

#endif // _VirtualRobot_RuntimeEnv_h_
