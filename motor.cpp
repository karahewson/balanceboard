/** @file motor.cpp
 *    This file contains a class which sets up the the PID controller and a task
 *    which calculates the PID control gain based on the current angle and desired angle.
 * 
 *  @author Kara Hewson
 *  @date   2020-Oct-22 Original file
 */

#include "motor.h"
#include "baseshare.h"
#include "taskqueue.h"
#include "taskshare.h"

extern Share<float> angle; // get access to the external share

/** @brief   Sets up the gains for a PID controller object. 
 *  @details Sets up the gains for a PID controller object and initializes variables
 *           that are used in the calculation to find the gain.
 *
 *  @param   kp is the proportional gain
 *  @param   ki is the integral gain
 *  @param   kd is the derivative gain
 */
Controller::Controller (float kp, float ki, float kd)
{
    // Save the paramters, which will evaporate when the constructor exits

    Kp = kp;        // Proportional Gain
    Ki = ki;        // Integral Gain
    Kd = kd;        // Derivative gain

    P_control = 0;  // Proportional Control
    I_control = 0;  // Integral Control
    D_control = 0;  // Derivative Control

    pre_error = 0;   // Initialize the previous error to be zero
    
}

/** @brief   Task which calculates the PID control gain based on the current angle and desired angle.  
 *  @details This task runs at precise interfals using @c vTaskDelayUntil() and
 *           calculates the proportional, derivative, and integral gain based on the error of the
 *           desired angle and measured angle from the IMU. The controller will saturate the gain
 *           if it is greater than 255 (100%). The gain is then returned.
 *
 */
 float Controller::pid (void)
{
    float PID_control;        // The gain of the PID control
    float desired_angle = -4; // Our desired angle
    float cur_angle;          // Set up the variable for storing the current angle 
    float Time = 5;           // The time between each measurement for the current angle 
 
    angle.get(cur_angle);    // get the current angle of the IMU
    
    float error = desired_angle - cur_angle; // the error between our desired angle and measured current angle

    P_control = Kp * error;                      //get P control value
    I_control += Ki * error * Time;              //get I control value
    D_control = Kd * (error - pre_error) / Time; //get D control value


    PID_control = abs(P_control + I_control + D_control); // get PID control value
    
    if (PID_control > 255) // Saturate the gain/duty cycle if its above 255 (which correlates to 100%)
    {
        PID_control = 255;
    }
    
    pre_error = error; // store the previous error 

    return PID_control; // return the PID control gain which will be used to set the duty cycle

}