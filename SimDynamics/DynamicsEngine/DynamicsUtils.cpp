
#include "DynamicsUtils.h"

#include <iostream>
#include <cmath>

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
    reset (gainP,gainI,gainD);
}

void PIDController::reset( double gainP, double gainI, double gainD )
{
    errorSum = 0.0;
    lastError = 0.0;
    this->gainP = gainP;
    this->gainI = gainI;
    this->gainD = gainD;
}


void PIDController::debug()
{
	std::cout << "error sum: " << errorSum
			  << " last error: " << lastError
			  << " last output: " << lastOutput
			  << std::endl;
}

void PIDController::getPID( double &storeP, double &storeI, double &storeD )
{
    storeP = gainP;
    storeI = gainI;
    storeD = gainD;
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

VelocityMotorController::VelocityMotorController(double maxVelocity, double maxAcceleration)
    : positionController(15.0, 0.0, 0.0)
, velocityController(15.0, 0.0, 0.0)
, maxVelocity(maxVelocity)
, maxAcceleration(maxAcceleration)
, velocity(0)
{
}
void VelocityMotorController::setCurrentVelocity(double vel)
{
    velocity = vel;
}

double VelocityMotorController::update(double positionError, double targetVelocity, ActuationMode actuation, double dt)
{
	double posUpdate = 0.0;
	if (actuation.modes.position)
	{
		posUpdate = positionController.update(positionError, dt);
	}

	double output = 0.0f;
	if (actuation.modes.position && actuation.modes.velocity)
		output = posUpdate + targetVelocity;
	else if (actuation.modes.position)
		output = posUpdate;
	else if (actuation.modes.velocity)
    {
        double velError = targetVelocity - velocity;
        output = velocityController.update(velError, dt);
    }

	if (maxVelocity > 0.0 && fabs(output) > maxVelocity)
	{
		double sign = output > 0 ? 1.0 : -1.0;
		output = sign * maxVelocity;
	}

	if (maxAcceleration > 0.0 && fabs(output - velocity)/dt > maxAcceleration)
	{
		double sign = (output - velocity) > 0 ? 1.0 : -1.0;

		output = velocity + sign * maxAcceleration*dt;
	}

	velocity = output;

	return output;
}

void VelocityMotorController::reset()
{
	positionController.reset();
}

void VelocityMotorController::reset( double pid_pos_gainP, double pid_pos_gainI, double pid_pos_gainD )
{
    positionController.reset(pid_pos_gainP,pid_pos_gainI,pid_pos_gainD);
}

void VelocityMotorController::getPosPID(double &storeP, double &storeI, double &storeD)
{
    positionController.getPID(storeP,storeI,storeD);
}

void VelocityMotorController::debug()
{
	positionController.debug();
}



}
