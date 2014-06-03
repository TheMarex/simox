
#include "DynamicsUtils.h"

#include <iostream>

namespace SimDynamics {
PIDController::PIDController(float gainP, float gainI, float gainD)
: gainP(gainP)
, gainI(gainI)
, gainD(gainD)
, errorSum(0)
, lastError(0)
{
}

float PIDController::update(float error, float dt)
{
	float p = error * gainP;
	errorSum += error * dt;
	float i = errorSum * gainI;
	float d = (error - lastError)/dt * gainD;
	lastError = error;

	float output = (p + i + d);
	lastOutput = output;
	return output;
}

void PIDController::reset()
{
	errorSum = 0.0;
	lastError = 0.0;
}

void PIDController::debug()
{
	std::cout << "error sum: " << errorSum
			  << " last error: " << lastError
			  << " last output: " << lastOutput
			  << std::endl;
}

TorqueMotorController::TorqueMotorController()
: positionController(0.5, 0.05, 0.0)
, velocityController(1.0, 0.05, 0.0)
, torqueController(1.0, 0.05, 0.0)
{
}

TorqueMotorController::TorqueMotorController(const PIDController& positionController,
				const PIDController& velocityController,
				const PIDController& torqueController)
: positionController(positionController)
, velocityController(velocityController)
, torqueController(torqueController)
{
}


float TorqueMotorController::update(float positionError, float velocityError, float torqueError, ActuationMode actuation, float dt)
{
	float posUpdate = 0.0;
	float velUpdate = 0.0;
	float torqueUpdate = 0.0;
	if (actuation.modes.position)
		posUpdate = positionController.update(positionError, dt);
	else
		positionController.reset();

	if (actuation.modes.velocity)
		velUpdate = velocityController.update(velocityError + posUpdate, dt);
	else
		velocityController.reset();

	if (actuation.modes.torque)
		torqueUpdate = torqueController.update(torqueError + velUpdate, dt);
	else
		torqueController.reset();

	if (actuation.modes.position && actuation.modes.velocity && actuation.modes.torque)
		return torqueUpdate;
	if (actuation.modes.position && actuation.modes.velocity)
		return velUpdate;
	if (actuation.modes.position)
		return posUpdate;
	if (actuation.modes.velocity)
		return velUpdate;
	if (actuation.modes.torque)
		return torqueUpdate;

	return 0.0f;
}

VelocityMotorController::VelocityMotorController()
: positionController(100.0, 50.0, 0.0)
{
}

VelocityMotorController::VelocityMotorController(const PIDController& positionController)
: positionController(positionController)
{
}


float VelocityMotorController::update(float positionError, float targetVelocity, ActuationMode actuation, float dt)
{
	float posUpdate = 0.0;
	if (actuation.modes.position)
		posUpdate = positionController.update(positionError, dt);
	else
		positionController.reset();

	float output = 0.0f;
	if (actuation.modes.position && actuation.modes.velocity)
		output = posUpdate + targetVelocity;
	else if (actuation.modes.position)
		output = posUpdate;
	else if (actuation.modes.velocity)
		output = targetVelocity;

	return output;
}

void VelocityMotorController::reset()
{
	positionController.reset();
}

void VelocityMotorController::debug()
{
	positionController.debug();
}
}
