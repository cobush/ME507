/** @file task_pwm_ramp.cpp
 *    This file contains a task which ramps a PWM duty cycle up and down to
 *    make an LED get steadily brighter and dimmer or a DC motor make really 
 *    annoying buzzing sounds.
 *
 *  @author JR Ridgely
 *  @date   2020-Nov-20 Original file
 */
#include <Arduino.h>
#include <PrintStream.h>
#if (defined STM32L4xx || defined STM32F3xx)
    #include <STM32FreeRTOS.h>
#endif
#include <taskshare.h>
#include <taskqueue.h>
#include <baseshare.h>
#include <CollectData.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <Wire.h>


// int16_t ax;
// int16_t ay;
// int16_t az;
// int16_t gx;
// int16_t gy;
// int16_t gz;
// uint8_t ADDR = 0;

/** @brief   Task which makes a PWM increase and decrease its duty cycle.
 *  @details This task runs at precise interfals using @c vTaskDelayUntil() and
 *           makes a PWM signal ramp up and down. It demonstrates a simple 
 *           finite state machine of the programming variety. If connected to
 *           an LED, it makes a sort of "heartbeat" effect; if connected to a
 *           motor driver, it gets annoying really quickly. 
 * 
 *           The state machine has the following states:
 *           * 0: Ramping Up - Increasing duty cycle of PWM
 *           * 1: Ramping Down - Decreasing duty cycle
 *
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void CollectData (void* p_params)
{

}