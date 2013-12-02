
#include "ForceTorqueSensor.h"
#include "ForceTorqueSensorFactory.h"
#include "../XML/BaseIO.h"

using namespace boost;

namespace VirtualRobot {

ForceTorqueSensor::ForceTorqueSensor(RobotNodeWeakPtr robotNode,
                const std::string &name) : Sensor(robotNode,name),
    forceTorqueValues(6)
{
    forceTorqueValues.setZero();
}


ForceTorqueSensor::~ForceTorqueSensor()
{
}

const Eigen::VectorXf &ForceTorqueSensor::getForceTorque()
{
    return forceTorqueValues;
}



void ForceTorqueSensor::print( bool printChildren, bool printDecoration ) const
{
	if (printDecoration)
        cout << "******** ForceTorqueSensor ********" << endl;
	Sensor::print(printChildren,false);
}


SensorPtr ForceTorqueSensor::_clone(const RobotNodePtr newRobotNode, const VisualizationNodePtr visualizationModel, float scaling)
{
    SensorPtr result(new ForceTorqueSensor(newRobotNode,name));
	return result;
}


std::string ForceTorqueSensor::toXML(const std::string &modelPath, int tabs)
{
    std::stringstream ss;
    std::string t = "\t";
    std::string pre = "";
    for (int i=0;i<tabs;i++)
        pre += t;
    ss << pre << "<Sensor type='" << ForceTorqueSensorFactory::getName() << "'/>" << endl;
	std::string pre2 = pre + t;


    return ss.str();
}

void ForceTorqueSensor::updateSensors(const Eigen::VectorXf &newForceTorque)
{
    forceTorqueValues = newForceTorque;
}

} // namespace VirtualRobot
