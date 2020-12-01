/** @file main.cpp
 *    This file contains two tasks. A motor task which runs the motors with a PID controller
 *    and an IMU task that gets the angle of the IMU. This is to run a DIY one wheel electric
 *    skateboard that currently only runs forward.
 *
 *  @author  Sammy Tran
 *  @author  Spencer Grenley
 *  @author  Kara Hewson
 * 
 *  @date    28 Sep 2020 Original file
 *  @date    12 Nov 2020 Updated file to have a PID controller and run a motor in forward and reverse
 *  @date    30 Nov 2020 Added code for the momentary switch and updated pins for ESC
 */

#if (defined STM32L4xx || defined STM32F4xx)
    #include <STM32FreeRTOS.h>
#endif
#include "Arduino.h"
#include "PrintStream.h"
#include "baseshare.h"
#include "taskshare.h"
#include "taskqueue.h"
#include "task_inclinometer.h"
#include "motor.h"

/// Share that carries an integer from user interface to simulation task
Share<float> angle ("Angle");

/** @brief   Task which runs a motor.
 *  @details This task runs at precise interfals using @c vTaskDelayUntil() and
 *           runs a motor whose duty cycle is controlled by a PID controller.
 * 
 *           The state machine has the following states:
 *           0: Motors stopped - sets the duty of the motors to zero
 *           1: Run motors - runs the motors with a PID controller
 *           
 *  @param   p_params A pointer to 0
 * function parameters which we don't use.
 */
void task_sim (void* p_params)
{
    (void)p_params;                           // Shuts up a compiler warning

    // Set up the variables of the simulation here
    const TickType_t sim_period = 10;         // RTOS ticks (ms) between runs
    float pitch = 0;                          // Set up the variable for storing the current angle 
    float gain;                               // Set up the variable for storing the gain
    uint16_t state = 0;                       // Set up the initial state of the motor
    bool button_pressed = 0;

    // Initialise the xLastWakeTime variable with the current time
    // It will be used to run the task at precise intervals
    TickType_t xLastWakeTime = xTaskGetTickCount(); 

    // Setting up the pins to the ESC to be outputs
    pinMode(D13, OUTPUT);
    pinMode(D12, OUTPUT);
    pinMode(D6, OUTPUT);
    pinMode(D5, OUTPUT);
    pinMode(D4, OUTPUT);
    pinMode(D3, OUTPUT);
    pinMode(D1, INPUT);

    // pin A3 to CONTROL INPUT - where we set the duty cycle
    digitalWrite(D13, HIGH);                   // Connect to DIRECTION/PHASE   , Setting up phase pin to have the motor be in forward direction
    digitalWrite(D12, LOW);                    // Connect to BRAKE             , Setting up the braking pin to be initially off (brake activated)
    digitalWrite(D5, LOW);                     // Connect to DIRECTION/PHASE   , Setting up phase pin to have the motor be in forward direction
    digitalWrite(D4, LOW);                     // Connect to ENABLE            , Setting up the braking pin to be initially off (brake activated)

    // Set up a PID control object with gains for kp, ki, kd
    Controller my_pid(38, .001, 0);

    for (;;)
    {
        angle.get(pitch);    // get the current angle of the IMU
        gain = my_pid.pid(); // get the gain of the PID controller to be used as the duty cycle
        
        if (state = 0)
        {
            // Activate the brakes
            digitalWrite(D12, LOW); 
            digitalWrite(D4, LOW);
            
            button_pressed = digitalRead(D1);
            
            if (button_pressed == true)
            {
                state = 1;
            }
        
        }
        
        else if (state = 1)
        {
            // Turn off the brakes on the motors
            digitalWrite(D12, HIGH); 
            digitalWrite(D4, HIGH);
            button_pressed = digitalRead(D1);
            
            if (button_pressed == true)
            {
                if (pitch <= -4) // motors do not run when we are less than -7 degrees
                {     
                    analogWrite (D6, 0);  // sets the Pulse Width Modulated (PWM) duty cycle of pin A3 on the Nucleo
                    analogWrite (D3, 0);  // sets the Pulse Width Modulated (PWM) duty cycle of pin A3 on the Nucleo
                }
                else
                {              
                    //Serial << endl << endl << "Motor 1:" << gain << endl;
                    analogWrite (D6, gain);  // sets the Pulse Width Modulated (PWM) duty cycle of pin A3 on the Nucleo
                    analogWrite (D3, gain);  // sets the Pulse Width Modulated (PWM) duty cycle of pin A3 on the Nucleo
                
                }
            }    
            
            else
            {
                state = 0;
            }    
        }
        

        // This type of delay waits until it has been the given number of RTOS
        // ticks since the task previously began running. This prevents timing
        // inaccuracy due to not accounting for how long the task took to run
        vTaskDelayUntil (&xLastWakeTime, sim_period);
        
    }
}


/** @brief   Arduino setup function which runs once at program startup.
 *  @details This function sets up a serial port for communication and creates
 *           the tasks which will be run.
 */
void setup () 
{
    // Start the serial port, wait a short time, then say hello. Use the
    // non-RTOS delay() function because the RTOS hasn't been started yet
    Serial.begin (115200);
    delay (2000);
    Serial << endl << endl << "ME507 UI Lab Starting Program Spinning a Motor" << endl;


    // Create a task which runs the motors
    xTaskCreate (task_sim,
                 "Simul.",
                 1024,                            // Stack size
                 NULL,
                 4,                               // Priority
                 NULL);            
    
    // Create a task which calculates the current angle from an IMU
    xTaskCreate (task_inc_angle,
                 "User Int.",                     // Name for printouts
                 1000,                            // Stack size
                 NULL,                            // Parameters for task fn.
                 5,                               // Priority
                 NULL);                           // Task handle
              

    // If using an STM32, we need to call the scheduler startup function now;
    // if using an ESP32, it has already been called for us
    #if (defined STM32L4xx || defined STM32F4xx)
        vTaskStartScheduler ();
    #endif
}


/** @brief   Arduino's low-priority loop function, which we don't use.
 *  @details A non-RTOS Arduino program runs all of its continuously running
 *           code in this function after @c setup() has finished. When using
 *           FreeRTOS, @c loop() implements a low priority task on most
 *           microcontrollers, and crashes on some others, so we'll not use it.
 */
void loop () 
{
}
