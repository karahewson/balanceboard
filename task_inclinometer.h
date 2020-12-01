/** @file dissect.h
 *    This task calculates the inclination angle using a function called getAngleX()
 *    using the driver mpu6050. After calculating the inclination angle, this task
 *    puts the angle into a share so it can be accessed by other tasks.
 * 
 *  @author Kara Hewson
 *  @date 2020-Nov-1
 */

#include <Arduino.h>
#include <PrintStream.h>
#include <mpu6050.h>
#include <taskqueue.h>
#if (defined STM32L4xx || defined STM32F4xx)
    #include <STM32FreeRTOS.h>
#endif


/// Function that takes apart a number and prints what it finds in decimal, binary, and hexadecimal formats.
/// The function also lists the decimal meaning of each of the bits in the number which are ones.
void task_inc_angle (void* p_param);