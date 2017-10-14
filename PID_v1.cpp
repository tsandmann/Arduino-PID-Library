/**
 * \file   PID_v1.cpp
 * \author Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * \author Timo Sandmann
 * \date   04.06.2017
 * \brief  Arduino PID Library for use with FreeRTOS - Version 1.2.0
 * \see    https://github.com/br3ttb/Arduino-PID-Library
 *
 * based on Arduino PID Library - Version 1.1.1 by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com, licensed under a GPLv3 License
 */

#include <FreeRTOS.h>
#include <task.h>
#include "PID_v1.h"


PID::PID(pid_t& Input, pid_t& Output, pid_t& Setpoint, const pid_t Kp, const pid_t Ki, const pid_t Kd, const bool Direction) :
		myInput(Input), myOutput(Output), mySetpoint(Setpoint),	outMin(0), outMax(255), inAuto(false), controllerDirection(Direction), sampleTime(100) {
	SetTunings(Kp, Ki, Kd);

	const auto ms(xTaskGetTickCount() * static_cast<uint32_t>(portTICK_PERIOD_US) / 1000UL);
	lastTime = ms - sampleTime;

	Initialize();
}

bool PID::Compute() {
	if (! inAuto) {
		return false;
	}

	const auto now_ms(xTaskGetTickCount() * static_cast<uint32_t>(portTICK_PERIOD_US) / 1000UL);
	const auto timeChange(now_ms - lastTime);

	if (timeChange >= sampleTime) {
		/* Compute all the working error variables */
		const auto error(mySetpoint - myInput);
		ITerm += (ki * error);
		if (ITerm > outMax) {
			ITerm = outMax;
		} else if (ITerm < outMin) {
			ITerm = outMin;
		}
		const auto dInput(myInput - lastInput);

		/* Compute PID output */
		auto output(kp * error + ITerm - kd * dInput);

		if (output > outMax) {
			output = outMax;
		} else if (output < outMin) {
			output = outMin;
		}
		myOutput = output;

		/* Remember some variables for next time */
		lastInput = myInput;
		lastTime = now_ms;

		return true;
	} else {
		return false;
	}
}

void PID::SetTunings(const pid_t Kp, const pid_t Ki, const pid_t Kd) {
	if (Kp < 0. || Ki < 0. || Kd < 0.) {
		return;
	}

	dispKp = Kp;
	dispKi = Ki;
	dispKd = Kd;

	const pid_t SampleTimeInSec((static_cast<pid_t>(sampleTime)) / 1000.);
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;

	if (! controllerDirection) {
		kp = (0. - kp);
		ki = (0. - ki);
		kd = (0. - kd);
	}
}

void PID::SetSampleTime(const uint16_t NewSampleTime) {
	const auto ratio(static_cast<pid_t>(NewSampleTime) / static_cast<pid_t>(sampleTime));
	ki *= ratio;
	kd /= ratio;
	sampleTime = NewSampleTime;
}

void PID::SetOutputLimits(const pid_t Min, const pid_t Max) {
	if (Min >= Max) {
		return;
	}
	outMin = Min;
	outMax = Max;

	if (inAuto) {
		if (myOutput > outMax) {
			myOutput = outMax;
		} else if (myOutput < outMin) {
			myOutput = outMin;
		}

		if (ITerm > outMax) {
			ITerm = outMax;
		} else if (ITerm < outMin) {
			ITerm = outMin;
		}
	}
}

void PID::SetMode(const Modes NewMode) {
	const bool newAuto(NewMode == Modes::AUTOMATIC);
	if (newAuto && ! inAuto) {
		/* we just went from manual to auto mode */
		Initialize();
	}
	inAuto = newAuto;
}

void PID::Initialize() {
	ITerm = myOutput;
	lastInput = myInput;
	if (ITerm > outMax) {
		ITerm = outMax;
	} else if (ITerm < outMin) {
		ITerm = outMin;
	}
}

void PID::SetControllerDirection(const bool Direction) {
	if (inAuto && Direction != controllerDirection) {
		kp = (0. - kp);
		ki = (0. - ki);
		kd = (0. - kd);
	}
	controllerDirection = Direction;
}
