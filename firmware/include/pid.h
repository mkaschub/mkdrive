/**
 * Set the setpoint for the PID controller
 *
 * @param  target The target (setpoint), the PID controller tries to reach
 */
void pidSetpoint(float target);

/**
 * Get setpoint of PID controller
 *
 * @return The target (setpoint), the PID controller tries to reach
 */
float pidGetSetpoint();

/**
 * Compute new control value of the PID controller
 * 
 * This algorithm does not need to be called in fixed intervals, 
 * because it keeps track of the time that passed since the last 
 * computation. 
 * Time granularity ist milli seconds (Arduino millis()), therefore
 * the computation will be performed max. once every milli second.
 * 
 * @param  current The current process value (e.g. the measured position of the motor )
 * @return New control value for the process. (e.g. a PWM value) 
 */
float pidCompute(float current);

