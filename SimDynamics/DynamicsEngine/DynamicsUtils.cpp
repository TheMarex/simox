
#include "DynamicsUtils.h"

#include <iostream>

namespace SimDynamics {
PIDController::PIDController(double gainP, double gainI, double gainD)
: gainP(gainP)
, gainI(gainI)
, gainD(gainD)
, errorSum(0)
, lastError(0)
{
}

double PIDController::update(double error, double dt)
{
	double p = error * gainP;
	errorSum += error * dt;
	double i = errorSum * gainI;
	double d = (error - lastError)/dt * gainD;
	lastError = error;

	double output = (p + i + d);
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


double TorqueMotorController::update(double positionError, double velocityError, double torqueError, ActuationMode actuation, double dt)
{
	double posUpdate = 0.0;
	double velUpdate = 0.0;
	double torqueUpdate = 0.0;
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
: positionController(100.0, 10.0, 0.0)
{
}

VelocityMotorController::VelocityMotorController(const PIDController& positionController)
: positionController(positionController)
{
}


double VelocityMotorController::update(double positionError, double targetVelocity, ActuationMode actuation, double dt)
{
	double posUpdate = 0.0;
	if (actuation.modes.position)
		posUpdate = positionController.update(positionError, dt);
	//else
	//	positionController.reset();

	double output = 0.0f;
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
