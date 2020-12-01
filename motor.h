/** @file motor.h
 *    This file contains a class which sets up the the PID controller and a task
 *    which calculates the PID control gain based on the current angle and desired angle.
 * 
 *  @author Kara Hewson
 *  @date   2020-October-22 Original file
 */

#include "Arduino.h"
#if (defined STM32L4xx || defined STM32F4xx)
    #include <STM32FreeRTOS.h>
#endif
#include "PrintStream.h"

class Controller
{
    protected:
        float Kp;            ///< The proportional gain
        float Ki;            ///< The integral gain
        float Kd;            ///< The derivative gain
        float pre_error;     ///< The previous error between current measured value and reference value
        float P_control, I_control, D_control; ///< The propotional, integral, and derivative control value, respectively

    public:
        // A Controller class that uses PID control, it is sent a proportional, integral, and derivative gain 
        Controller (float Kp, float Ki, float Kd);
        
        float pid (void);     // Run a pid controller
};