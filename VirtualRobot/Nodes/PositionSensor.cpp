
#include "PositionSensor.h"

using namespace boost;

namespace VirtualRobot {

PositionSensor::PositionSensor(	RobotNodeWeakPtr robotNode, 
				const std::string &name,
				VisualizationNodePtr visualization,
				const Eigen::Matrix4f &rnTrafo 
				) : Sensor(robotNode,name,visualization,rnTrafo)
{

}


PositionSensor::~PositionSensor()
{
}



void PositionSensor::print( bool printChildren, bool printDecoration ) const
{
	if (printDecoration)
		cout << "******** PositionSensor ********" << endl;
	Sensor::print(printChildren,false);
}


SensorPtr PositionSensor::_clone(const RobotNodePtr newRobotNode, const VisualizationNodePtr visualizationModel, float scaling)
{
	THROW_VR_EXCEPTION_IF(scaling<0,"Scaling must be >0");
	Eigen::Matrix4f rnt = rnTransformation;
	rnt.block(0,3,3,1) *= scaling;
	SensorPtr result(new PositionSensor(newRobotNode,name,visualizationModel,rnt));
	return result;
}

} // namespace VirtualRobot
