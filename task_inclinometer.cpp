/** @brief   Task which calculates the inclination angle X.
 *  @details This task calculates the inclination angle using a function called getAngleX()
 *           using the driver mpu6050. After calculating the inclination angle, this task
 *           puts the angle into a share so it can be accessed by other tasks.
 * 
 *  @param   p_params A pointer to function parameters which we don't use.
 */

#if (defined STM32L4xx || defined STM32F4xx)
#include <STM32FreeRTOS.h>
#endif
#include <Arduino.h>
#include <PrintStream.h>
#include "baseshare.h"
#include "taskshare.h"
#include "taskqueue.h"
#include "mpu6050.h"
#include "task_inclinometer.h"


// To get access to the share declared in main
extern Share<float> angle;

void task_inc_angle (void* p_params)
{
  (void)p_params;                           // Shuts up a compiler warning

  // Create an accelerometer object using driver found onlin
  MPU6050 mpu6050(Wire);   

  const TickType_t sim_period = 5;         // RTOS ticks (ms) between runs
  
  // Initialize the xLastWakeTime variable with the current time
  // It will be used to run the task at precise intervals
  TickType_t xLastWakeTime = xTaskGetTickCount();                

  Wire.begin();                  // set up lines for communication
  mpu6050.begin();               // start communication with the IMU
  mpu6050.calcGyroOffsets(true); // Callibrate and calculate offsets for the gyroscope
  float inclination_angle;       // variable to hold inclination angle  


  for (;;)
  {
    mpu6050.update();            // to get current data of the IMU

    inclination_angle = mpu6050.getAngleX(); // get inclination angle in degrees
    
    //Serial << inclination_angle << endl;
  
    angle.put(inclination_angle);  // put the inclination angle into the queue
    
    // This type of delay waits until it has been the given number of RTOS
    // ticks since the task previously began running. This prevents timing
    // inaccuracy due to not accounting for how long the task took to run
    vTaskDelayUntil (&xLastWakeTime, sim_period);

  }  
}