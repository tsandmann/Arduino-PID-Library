/**
 * \file   PID_v1.h
 * \author Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * \author Timo Sandmann
 * \date   04.06.2017
 * \brief  Arduino PID Library for use with FreeRTOS - Version 1.2.0
 * \see    https://github.com/br3ttb/Arduino-PID-Library
 *
 * based on Arduino PID Library - Version 1.1.1 by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com, licensed under a GPLv3 License
 */

#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.0

#include <cstdint>

/**
 * PID controller class
 */
class PID {
public:
	using pid_t = float; /**< datatype for internal storage and calculations, float or double */

	/**
	 * Possible modes for PID controller
	 */
	enum class Modes : uint8_t {
		MANUAL = 0,
		AUTOMATIC = 1,
	};

	/**
	 * \param Input Reference to the input variable to be used
	 * \param Output Reference to the output variable to be used
	 * \param Setpoint Reference to the setpoint variable to be used
	 * \param Kp (P)roportional tuning parameter
	 * \param Ki (I)ntegral tuning parameter
	 * \param Kd (D)erivative tuning parameter
	 * \param Direction true: output will increase when error is positive; false: the opposite
	 */
	PID(pid_t &Input, pid_t &Output, pid_t &Setpoint, const pid_t Kp, const pid_t Ki, const pid_t Kd, const bool Direction);

	/**
	 * \brief Performs the PID calculation.
	 * \return Returns true when the output is computed, false when nothing has been done.
	 * \note Should be called every time loop() cycles. ON/OFF and calculation frequency can be set using SetMode and SetSampleTime respectively.
	 */
	bool Compute();

	/**
	 * \brief Sets PID to either manual or automatic mode
	 * \param NewMode Mode to set, Modes::MANUAL or Modes::AUTOMATIC
	 *
	 * Allows the controller Mode to be set to manual or automatic when the transition from manual to auto occurs, the controller is automatically initialized.
	 */
	void SetMode(const Modes NewMode);

	/**
	 * \brief Clamps the output to a specific range
	 * \param Min Minimum output value
	 * \param Mac Maximum output value
	 * \note 0-255 by default
	 */
	void SetOutputLimits(const pid_t Min, const pid_t Max);

	/**
	 * \brief Gives the user the option of changing tunings during runtime for Adaptive control
	 * \param Kp (P)roportional tuning parameter
	 * \param Ki (I)ntegral tuning parameter
	 * \param Kd (D)erivative tuning parameter
	 *
	 * This method allows the controller's dynamic performance to be adjusted.
	 */
	void SetTunings(const pid_t Kp, const pid_t Ki, const pid_t Kd);

	/**
	 * \brief Sets the Direction, or "Action" of the controller
	 * \param Direction true: output will increase when error is positive; false: the opposite
	 *
	 * The PID will either be connected to a DIRECT acting process (+Output leads to +Input) or a REVERSE acting process (+Output leads to -Input).
	 * We need to know which one, because otherwise we may increase the output when we should be decreasing.
	 */
	void SetControllerDirection(const bool Direction);

	/**
	 * \brief Sets the period, in milliseconds, at which the calculation is performed
	 * \param NewSampleTime Sample time to use in milliseconds
	 * \note default is 100
	 */
	void SetSampleTime(const uint16_t NewSampleTime);

	/**
	 * Gets the proportional tuning parameter
	 * \return Kp paramter
	 */
	auto GetKp() const {
		return dispKp;
	}

	/**
	 * Gets the integral tuning parameter
	 * \return Ki paramter
	 */
	auto GetKi() const {
		return dispKi;
	}

	/**
	 * Gets the derivative tuning parameter
	 * \return Kd paramter
	 */
	auto GetKd() const {
		return dispKd;
	}

	/**
	 * Gets the mode the controller is in
	 * \return Currently set mode
	 */
	auto GetMode() const {
		return inAuto ? Modes::AUTOMATIC : Modes::MANUAL;
	}

	/**
	 * Gets the direction the controller is operating with
	 * \return Currently set direction
	 */
	auto GetDirection() const {
		return controllerDirection;
	}

private:
	pid_t &myInput; /**< Pointer to the input variable */
	pid_t &myOutput; /**< Pointer to the output variable */
	pid_t &mySetpoint; /**< Pointer to the setpoint variable */
	pid_t outMin, outMax; /**< Min / max values for outputs */
	pid_t ITerm, lastInput; /**< internal data for I-term and last input value */

	pid_t kp; /**< (P)roportional tuning parameter */
	pid_t ki; /**< (I)ntegral tuning parameter */
	pid_t kd; /**< (D)erivative tuning parameter */

	bool inAuto; /**< Mode of controller (automatic or manual) */
	bool controllerDirection; /**< Direction, or "Action" of the controller. true: direct, false: reverse */
	uint16_t sampleTime; /**< The period, in milliseconds, at which the calculation is performed */
	uint32_t lastTime; /**< Timestamp of last PID calculation */

	/* we'll hold on to the tuning parameters in user-entered format for display purposes */
	pid_t dispKp; /**< (P)roportional tuning parameter as set by user */
	pid_t dispKi; /**< (I)ntegral tuning parameter as set by user */
	pid_t dispKd; /**< (D)erivative tuning parameter as set by user */

	/**
	 * Does all the things that need to happen to ensure a bumpless transfer from manual to automatic mode.
	 */
	void Initialize();
};
#endif /* PID_v1_h */
