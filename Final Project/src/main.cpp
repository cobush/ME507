
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <MPU9250.h>
#include <PrintStream.h>
#if (defined STM32L4xx || defined STM32F4xx)
    #include <STM32FreeRTOS.h>
#endif

#include <taskqueue.h>
#include <taskshare.h>
#include <baseshare.h>
#include <task.h>
#include "Controller.h"

// define general variables to be used when pulling data from shares
bool one_bit;
uint8_t eight_bit;
uint16_t sixteen_bit;

// define variables to control Cube reaction
uint16_t mtr_duty;
bool mtr_dir;
uint16_t cube_ang;
uint16_t imu;
uint16_t generic_speed;
uint16_t enca_i;
uint16_t enca_f;
uint16_t enca_diff;
uint16_t time_diff;

// Shares and queues go here
Share<uint8_t> duty_cycle ("Power");
Share<bool> direction ("Direction");
Queue<uint16_t [6]> gyro_data ("Current Gyro Data");
Share<uint16_t> motor_speed("Current Motor Speed");
Share<uint8_t> cube_angle ("Current Cube Angle off Vertical");
Share<uint8_t> state ("Current state of the ");


void PWM (void* p_params)
{
(void)p_params;

duty_cycle.get(eight_bit);
mtr_duty = eight_bit;
direction.get(one_bit);
mtr_dir = one_bit;

analogWrite(XX, mtr_duty);
digitalWrite(XX, mtr_dir);

}
void PID ( void* p_params)
{
(void)p_params;
uint16_t mtr_duty;
duty_cycle.put(mtr_duty);
}
void Calculator (void* p_params)
{
(void)p_params;
// Collect the current data
gyro_data.get(sixteen_bit);
imu = sixteen_bit;
motor_speed.get(sixteen_bit);
generic_speed = sixteen_bit;

// Calculate motor direction and cube angle to pass into PID
direction.put(mtr_dir);
cube_angle.put(cube_ang);
}
void Encoder (void* p_params)
{
(void)p_params;

// check size of timer variable
if (0 == configUSE_16_BIT_TICKS)
{

}
else
{
  /* code */
}

// collect data to calculate motor speed
TickType_t time_i, time_f;
time_i = xTaskGetTickCount();
enca_i = analogRead(XX);
time_f = xTaskGetTickCount();
enca_f = analogRead(XX);

// need two if-else clauses to caluclate magnitude of difference in encoder count
// and timer count that does not wack out when timer or encoder cycle through zero
generic_speed = (enca_diff)/(time_diff);

motor_speed.put(generic_speed);
}
void IMU(void* p_params)
{
(void)p_params;

}


void setup() 
{

  // put your setup code here, to run once:
  
    // Start the serial port, wait a short time, then say hello. Use the
    // non-RTOS delay() function because the RTOS hasn't been started yet
    Serial.begin (115200);
    delay (2000);
    Serial << endl << endl << "ME507 Cube Balance Intializing..." << endl;

    // Initialize the motor, encoder, and IMU

    // Initialize motor PWM and direction pins
    // Motor PWM pin
    pinMode(A3, OUTPUT);
    // Motor direction pin
    pinMode(A0, OUTPUT);
    
    // Initialize motor encoder pins
    // ENCA
    pinMode(PA15,INPUT);
    //ENCB
    pinMode(PA14, INPUT);

    // Initialize IMU pins
    // SDA pin
    pinMode(PA15, INPUT);
    // SDL pin
    pinMode(PA14, INPUT);
    


// Dfldjf
    xTaskCreate (PWM,
                 "PWM Output",                 // Name for printouts
                 1536,                            // Stack size
                 NULL,                            // Parameters for task fn.
                 4,                               // Priority
                 NULL);                           // Task handle
    xTaskCreate (PID,
                 "PWM Output",                 // Name for printouts
                 1536,                            // Stack size
                 NULL,                            // Parameters for task fn.
                 5,                               // Priority
                 NULL);                           // Task handle
    xTaskCreate (Calculator,
                 "PWM Output",                 // Name for printouts
                 1536,                            // Stack size
                 NULL,                            // Parameters for task fn.
                 3,                               // Priority
                 NULL);                           // Task handle
    xTaskCreate (Encoder,
                 "PWM Output",                 // Name for printouts
                 1536,                            // Stack size
                 NULL,                            // Parameters for task fn.
                 2,                               // Priority
                 NULL);                           // Task handle
    xTaskCreate (IMU,
                 "PWM Output",                 // Name for printouts
                 1536,                            // Stack size
                 NULL,                            // Parameters for task fn.
                 1,                               // Priority
                 NULL);                           // Task handle                 
    // If using an STM32, we need to call the scheduler startup function now;
    // if using an ESP32, it has already been called for us

    #if (defined STM32L4xx || defined STM32F4xx)
        vTaskStartScheduler ();
    #endif

    Serial << "Completed Initialization, please place the cube in the starting position." << endl;
}



void loop() 
{
  // put your main code here, to run repeatedly:
}