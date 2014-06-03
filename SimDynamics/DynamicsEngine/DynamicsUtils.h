#ifndef __DYNAMICS_UTILS__H__
#define __DYNAMICS_UTILS__H__

#include <iostream>

namespace SimDynamics {
class PIDController {
public:
	PIDController(float gainP, float gainI, float gainD);

	float update(float error, float dt);

	void reset();

	void debug();

private:
	float gainP;
	float gainI;
	float gainD;
	float errorSum;
	float lastError;
	float lastOutput;
};

// use bit field because enums are a pain
union ActuationMode {
	struct {
		unsigned char position:1;
		unsigned char velocity:1;
		unsigned char torque:1;
	} modes;
	unsigned char mode;
};

/**
 * For *torque* based motors.
 *
 * Position only:
 * position error --> [PID] --> joint
 *
 * Velocity only:
 * velocity error --> [PID] --> joint
 *
 * Torque only:
 * torque error --> [PID] --> joint
 *
 * Position + Velocity:
 *                       velocity error
 *                             |
 *                             v
 * position error -> [PID] -> (+) -> [PID] -> joint
 *
 * Position + Velocity + Torque:
 *                       velocity error  torque error
 *                             |               |
 *                             v               v
 * position error -> [PID] -> (+) -> [PID] -> (+) -> [PID] -> joint
 *
 */
class TorqueMotorController{
public:
	TorqueMotorController();
	TorqueMotorController(const PIDController& positionController,
					const PIDController& velocityController,
					const PIDController& torqueController);

	float update(float positionError, float velocityError, float torqueError, ActuationMode actuation, float dt);

private:
	PIDController positionController;
	PIDController velocityController;
	PIDController torqueController;
};

/**
 * For *velocity* based motors (where target velocity == actual velocity).
 * We use this for Bullet motors.
 *
 * Note: Torque is ignored. This controler returns *velocities*.
 *
 * Position only:
 * position error --> [PID] --> joint
 *
 * Velocity only:
 * target velocity --> joint
 *
 * Position + Velocity:
 *                       target velocity
 *                             |
 *                             v
 * position error -> [PID] -> (+) -> joint
 *
 */
class VelocityMotorController {
public:
	VelocityMotorController();

	VelocityMotorController(const PIDController& positionController);


	float update(float positionError, float targetVelocity, ActuationMode actuation, float dt);

	void reset();

	void debug();

private:
	PIDController positionController;
};
}

#endif
