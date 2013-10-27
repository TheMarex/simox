
#include "CameraSensor.h"

using namespace boost;

namespace VirtualRobot {

CameraSensor::CameraSensor(	RobotNodeWeakPtr robotNode, 
				const std::string &name,
				VisualizationNodePtr visualization,
				const Eigen::Matrix4f &rnTrafo 
				) : Sensor(robotNode,name,visualization,rnTrafo)
{

}


CameraSensor::~CameraSensor()
{
}



void CameraSensor::print( bool printChildren, bool printDecoration ) const
{
	if (printDecoration)
		cout << "******** CameraSensor ********" << endl;
	Sensor::print(printChildren,false);
}


SensorPtr CameraSensor::_clone(const RobotNodePtr newRobotNode, const VisualizationNodePtr visualizationModel)
{
	SensorPtr result(new CameraSensor(newRobotNode,name,visualizationModel,rnTransformation));
	return result;
}

} // namespace VirtualRobot
