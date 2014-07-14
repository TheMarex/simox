#ifndef __BULLET_LOGGER_H__
#define __BULLET_LOGGER_H__

#include <Eigen/Dense>

#include "BulletEngine.h"
#include "BulletRobot.h"

/*
 * Logger for a BulletRobot that logs target/actual position/velocity to a file.
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
	, logPath("")
	{
		engine->addExternalCallback(logCB, (void*) this);
	}

	~BulletRobotLogger()
	{
		if (logPath.size() > 0)
		{
			writeToFile(logPath);
		}
	}

	void setLogPath(const std::string& path)
	{
		logPath = path;
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
	std::vector<Eigen::VectorXf> actualJointTorquesLog;
	std::vector<Eigen::Matrix3Xf> actualJointForcesLog;
	std::vector<Eigen::Vector3f> actualCoMLog;
	std::vector<Eigen::Vector3f> actualCoMVelocityLog;
	std::vector<double> timestamps;
	double timestamp;
	bool running;
	int max_samples;
	std::string logPath;

	static void logCB(void* data, btScalar dt);
	void log(btScalar dt);
};

typedef boost::shared_ptr<BulletRobotLogger> BulletRobotLoggerPtr;

}

#endif
