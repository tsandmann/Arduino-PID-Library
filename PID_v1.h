/**
 * \file   PID_v1.h
 * \author Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * \author Timo Sandmann
 * \date   04.06.2017
 * \brief  Arduino PID Library for use in ct-Bot framework - Version 1.2.0
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
class Pid {
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
     * \param input Reference to the input variable to be used
     * \param output Reference to the output variable to be used
     * \param setpoint Reference to the setpoint variable to be used
     * \param kp (P)roportional tuning parameter
     * \param ki (I)ntegral tuning parameter
     * \param kd (D)erivative tuning parameter
     * \param direction true: output will increase when error is positive; false: the opposite
     */
    Pid(pid_t& input, pid_t& output, pid_t& setpoint, const pid_t kp, const pid_t ki, const pid_t kd, const bool direction);

    /**
     * \brief Performs the PID calculation.
     * \return Returns true when the output is computed, false when nothing has been done.
     * \note Should be called every time loop() cycles. ON/OFF and calculation frequency can be set using SetMode and SetSampleTime respectively.
     */
    bool compute();

    /**
     * \brief Sets PID to either manual or automatic mode
     * \param new_mode Mode to set, Modes::MANUAL or Modes::AUTOMATIC
     *
     * Allows the controller Mode to be set to manual or automatic when the transition from manual to auto occurs, the controller is automatically initialized.
     */
    void set_mode(const Modes new_mode);

    /**
     * \brief Clamps the output to a specific range
     * \param min Minimum output value
     * \param mac Maximum output value
     * \note 0-255 by default
     */
    void set_output_limits(const pid_t min, const pid_t max);

    /**
     * \brief Gives the user the option of changing tunings during runtime for Adaptive control
     * \param kp (P)roportional tuning parameter
     * \param ki (I)ntegral tuning parameter
     * \param kd (D)erivative tuning parameter
     *
     * This method allows the controller's dynamic performance to be adjusted.
     */
    void set_tunings(const pid_t kp, const pid_t ki, const pid_t kd);

    /**
     * \brief Sets the Direction, or "Action" of the controller
     * \param direction true: output will increase when error is positive; false: the opposite
     *
     * The PID will either be connected to a DIRECT acting process (+Output leads to +Input) or a REVERSE acting process (+Output leads to -Input).
     * We need to know which one, because otherwise we may increase the output when we should be decreasing.
     */
    void set_controller_direction(const bool direction);

    /**
     * \brief Sets the period, in milliseconds, at which the calculation is performed
     * \param NewSampleTime Sample time to use in milliseconds
     * \note default is 100
     */
    void set_sample_time(const uint16_t new_sample_time);

    /**
     * Gets the proportional tuning parameter
     * \return Kp paramter
     */
    auto get_kp() const {
        return disp_kp_;
    }

    /**
     * Gets the integral tuning parameter
     * \return Ki paramter
     */
    auto get_ki() const {
        return disp_ki_;
    }

    /**
     * Gets the derivative tuning parameter
     * \return Kd paramter
     */
    auto get_kd() const {
        return disp_kd_;
    }

    /**
     * Gets the mode the controller is in
     * \return Currently set mode
     */
    auto get_mode() const {
        return in_auto_ ? Modes::AUTOMATIC : Modes::MANUAL;
    }

    /**
     * Gets the direction the controller is operating with
     * \return Currently set direction
     */
    auto get_direction() const {
        return direction_;
    }

private:
    pid_t& input_; /**< Pointer to the input variable */
    pid_t& output_; /**< Pointer to the output variable */
    pid_t& setpoint_; /**< Pointer to the setpoint variable */
    pid_t out_min_, out_max_; /**< Min / max values for outputs */
    pid_t i_term_, last_input_; /**< internal data for I-term and last input value */

    pid_t kp_; /**< (P)roportional tuning parameter */
    pid_t ki_; /**< (I)ntegral tuning parameter */
    pid_t kd_; /**< (D)erivative tuning parameter */

    bool in_auto_; /**< Mode of controller (automatic or manual) */
    bool direction_; /**< Direction, or "Action" of the controller. true: direct, false: reverse */
    uint16_t sample_time_; /**< The period, in milliseconds, at which the calculation is performed */
    uint32_t last_time_; /**< Timestamp of last PID calculation */

    /* we'll hold on to the tuning parameters in user-entered format for display purposes */
    pid_t disp_kp_; /**< (P)roportional tuning parameter as set by user */
    pid_t disp_ki_; /**< (I)ntegral tuning parameter as set by user */
    pid_t disp_kd_; /**< (D)erivative tuning parameter as set by user */

    /**
     * Does all the things that need to happen to ensure a bumpless transfer from manual to automatic mode.
     */
    void initialize();
};
#endif /* PID_v1_h */
