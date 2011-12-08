#include "LinkedCoordinate.h"
#include "Robot.h"
#include <boost/format.hpp>

using namespace VirtualRobot;
using namespace boost;
using namespace std;



void LinkedCoordinate::set( const RobotNodePtr &frame,const Eigen::Matrix4f &pose) throw(VirtualRobotException) {
	if (!frame) 
		THROW_VR_EXCEPTION(format("RobotNodePtr not assigned (LinkedCoordinate::%1%)") % __func__);
	if (!this->robot->hasRobotNode( frame ) ) 
		THROW_VR_EXCEPTION(format("Robot node\"%1%\" not a member of robot \"%2%\" (LinkedCoordinate::%3%)") % frame->getName() % this->robot->getName() % __func__);
	this->pose = pose;
	this->frame = frame;
}


void LinkedCoordinate::set( const std::string &frame,const Eigen::Matrix4f &pose) throw(VirtualRobotException) {
	if (!this->robot->hasRobotNode( frame ) )
		THROW_VR_EXCEPTION(format("Robot node\"%1%\" not a member of robot \"%2%\" (LinkedCoordinate::%3%)") % frame % this->robot->getName() %__func__);
	this->set(this->robot->getRobotNode(frame),pose);
}


void LinkedCoordinate::set( const std::string &frame,const Eigen::Vector3f &coordinate) throw(VirtualRobotException) {
	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
	pose.block<3,1>(0,3) = coordinate;
	this->set(frame,pose);
}
	

void LinkedCoordinate::set( const RobotNodePtr &frame,const Eigen::Vector3f &coordinate) throw(VirtualRobotException){
	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
	pose.block<3,1>(0,3) = coordinate;
	this->set(frame,pose);
}


void LinkedCoordinate::changeFrame(const RobotNodePtr & destination) throw(VirtualRobotException){
	if (!destination) 
		THROW_VR_EXCEPTION(format("RobotNodePtr not assigned (LinkedCoordinate::%1%)") % __func__);
	if (!this->robot->hasRobotNode( destination) ) 
		THROW_VR_EXCEPTION(format("Robot node\"%1%\" not a member of robot \"%2%\" (LinkedCoordinate::%3%)") % destination->getName() % this->robot->getName() % __func__);
	
	if (!this->frame)
		this->pose = Eigen::Matrix4f::Identity();
	else
		this->pose = LinkedCoordinate::getCoordinateTransformation(this->frame,destination,this->robot) * this->pose;
	
	this->frame = destination;
}

void LinkedCoordinate::changeFrame(const std::string & destination) throw(VirtualRobotException){
	if (!this->robot->hasRobotNode( destination) ) 
		THROW_VR_EXCEPTION(format("Robot node\"%1%\" not a member of robot \"%2%\" (LinkedCoordinate::%3%)") % destination % this->robot->getName() % __func__);
	this->changeFrame(this->robot->getRobotNode(destination));
}

Eigen::Matrix4f LinkedCoordinate::getInFrame(const RobotNodePtr & destination) const  throw(VirtualRobotException){
	if (!destination) 
		THROW_VR_EXCEPTION(format("RobotNodePtr not assigned (LinkedCoordinate::%1%)") % __func__);
	if (!this->robot->hasRobotNode( destination) ) 
		THROW_VR_EXCEPTION(format("Robot node\"%1%\" not a member of robot \"%2%\" (LinkedCoordinate::%3%)") % destination->getName() % this->robot->getName() % __func__);
	return LinkedCoordinate::getCoordinateTransformation(this->frame,destination,this->robot) * this->pose;
}


Eigen::Matrix4f LinkedCoordinate::getInFrame(const std::string & destination) const  throw(VirtualRobotException){
	if (!this->robot->hasRobotNode( destination) ) 
		THROW_VR_EXCEPTION(format("Robot node\"%1%\" not a member of robot \"%2%\" (LinkedCoordinate::%3%)") % destination % this->robot->getName() % __func__);
	return LinkedCoordinate::getCoordinateTransformation(this->frame,this->robot->getRobotNode( destination),this->robot) * this->pose;
}



Eigen::Matrix4f LinkedCoordinate::getCoordinateTransformation(const RobotNodePtr &origin, 
		const RobotNodePtr &destination, const RobotPtr &robot){
	
	if (!destination) 
		THROW_VR_EXCEPTION(format("Destination RobotNodePtr not assigned (LinkedCoordinate::%1%)") % __func__);
	if (!origin) 
		THROW_VR_EXCEPTION(format("Origin RobotNodePtr not assigned (LinkedCoordinate::%1%)") % __func__);
	if (!robot) 
		THROW_VR_EXCEPTION(format("RobotPtr not assigned (LinkedCoordinate::%1%)") % __func__);
	if (!robot->hasRobotNode( origin) ) 
		THROW_VR_EXCEPTION(format("Origin robot node\"%1%\" not a member of robot \"%2%\" (LinkedCoordinate::%3%)") % origin->getName() % robot->getName() % __func__);
	if (!robot->hasRobotNode( destination) ) 
		THROW_VR_EXCEPTION(format("Destination robot node\"%1%\" not a member of robot \"%2%\" (LinkedCoordinate::%3%)") % destination->getName() % robot->getName() % __func__);
	
//	std::cout << "Destination: " << destination->getName() <<std::endl << "Origin: " << origin->getName() << std::endl;
//	std::cout << "Destination:\n" << destination->getGlobalPose() <<std::endl << "Origin:\n" << origin->getGlobalPose() << std::endl;
	return destination->getGlobalPose().inverse() * origin->getGlobalPose();
}
