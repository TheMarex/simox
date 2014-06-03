#ifndef __BULLET_LOGGER_H__
#define __BULLET_LOGGER_H__

#include <Eigen/Dense>

#include "BulletEngine.h"
#include "BulletRobot.h"

/*
 * Wrapper for a BulletRobot that logs target/actual position/velocity to a file.
 */
namespace SimDynamics
{

class SIMDYNAMICS_IMPORT_EXPORT BulletRobotLogger
{
public:
	BulletRobotLogger(BulletEnginePtr engine,
					  const BulletRobotPtr robot,
					  const VirtualRobot::RobotNodeSetPtr& jointNodes,
					  const VirtualRobot::RobotNodeSetPtr& bodyNodes)
	: robot(robot)
	, running(false)
	, jointNodes(jointNodes)
	, bodyNodes(bodyNodes)
	, max_samples(1024 * 1024)
	, timestamp(0.0f)
	{
		engine->addExternalCallback(logCB, (void*) this);
	}

	void writeToFile(const std::string& path);
	void startLogging();
	void stopLogging();

private:
	const BulletRobotPtr robot;
	VirtualRobot::RobotNodeSetPtr jointNodes;
	VirtualRobot::RobotNodeSetPtr bodyNodes;
	std::vector<Eigen::VectorXf> targetAngleLog;
	std::vector<Eigen::VectorXf> targetVelocityLog;
	std::vector<Eigen::VectorXf> actualAngleLog;
	std::vector<Eigen::VectorXf> actualVelocityLog;
	std::vector<Eigen::Vector3f> actualCoMLog;
	std::vector<Eigen::Vector3f> actualCoMVelocityLog;
	std::vector<float> timestamps;
	float timestamp;
	bool running;
	int max_samples;

	static void logCB(void* data, btScalar dt);
	void log(btScalar dt);
};

typedef boost::shared_ptr<BulletRobotLogger> BulletRobotLoggerPtr;

}

#endif
