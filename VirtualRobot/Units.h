#ifndef _VirtualRobot_Units_h_
#define _VirtualRobot_Units_h_

/**
* @package    VirtualRobot
* @author     Manfred Kroehnert <mkroehnert _at_ users dot sourceforge dot net>
* @copyright  2011 Manfred Kroehnert
*/

#include "VirtualRobotImportExport.h"
#include <string>
#include <algorithm>

namespace VirtualRobot {

class VIRTUAL_ROBOT_IMPORT_EXPORT Units
{
public:

	enum UnitsType
	{
		eAngle,
		eLength,
		eWeight,
		eIgnore
	};

	Units(const std::string& unitName) : unitString(unitName)
	{
		//std::transform(unitString.begin(), unitString.end(), unitString.begin(), ::tolower);
	}

	bool isRadian() {return ("radian" == unitString || "rad" == unitString);}
	bool isDegree() {return ("degree" == unitString || "deg" == unitString);}
	bool isAngle() {return (isRadian() ||  isDegree());}

	bool isMillimeter() {return ("mm" == unitString || "millimeter" == unitString);}
	bool isMeter() {return ("m" == unitString || "meter" == unitString);}
	bool isLength() {return (isMillimeter() ||  isMeter());}

	bool isGram() {return ("g" == unitString || "gram" == unitString);}
	bool isKilogram() {return ("kg" == unitString || "kilogram" == unitString);}
	bool isWeight() {return (isGram() ||  isKilogram());}

	bool isValid() {return (isLength() || isAngle() || isWeight());}

private:
	std::string unitString;
};

} // namespace VirtualRobot

#endif /* _VirtualRobot_Units_h_ */
