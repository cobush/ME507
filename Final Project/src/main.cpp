/** @file main.cpp
 *    This file contains a program that executes a series of tasks to balance a cube
 *    on one edge using an MPU9250 accelerometer and NUCLEO STM32L476RG microcontroller.
 *    The PID controller and MPU9250 driver are not the intellectual property of the group.
 *    See ownership for each file below:
 *    @file MPU9250.h
 *    @file MPU9250RegisterMap.h
 *    @file QuarternionFilter.h
 *
 *    @author hideakitai
 *    @source https://github.com/hideakitai/MPU9250.git
 * 
 *    @file MiniPID.cpp
 *    @file MiniPID.h
 * 
 *    @author tekdemo
 *    @source https://github.com/tekdemo/MiniPID.git
 *
 *  @author  Connor Bush
 *  @author  Joseph Heald
 *  @author  Blake Carbonneau
 *  @author  Giovanni Guerrero
 * 
 *  @date    15 NOV 2020 Original file
 *  @date    16 NOV 2020 Updated driver for MPU9250 and included MiniPID
 *  @date    21 NOV 2020 Added doxygen comments for easier use and grading
 *  @date    30 NOV 2020 Updated code after testing the physical cube
 *  @date    02 DEC 2020 Updated comments to reflect testing
 */

/* Include the necessary header files*/
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <MPU9250.h>
#include <MPU9250RegisterMap.h>
#include <QuaternionFilter.h>
#include <MiniPID.h>
#include <PrintStream.h>
#if (defined STM32L4xx || defined STM32F4xx)
    #include <STM32FreeRTOS.h>
#endif

#include <taskqueue.h>
#include <taskshare.h>
#include <baseshare.h>
#include <task.h>


/* Variables that will serve as temporaty storage before passing into shares*/
uint8_t mtr_duty;
bool mtr_dir;
float cube_ang;

/* Variable that serves as temporary storage for error value*/
uint16_t error;

/* Variables used to initialize the PID controller*/
uint8_t MAX_DUTY = 255; // Maximum duty cycle output*/
double VERTICAL = 45; // constant value set to zero the gyro on the vertical axis*/
double P = 1; // Proportional constant*/
double I = 1; // Integral constant*/
double D = 1; // Derivative constant*/
double F = 0; // Feed forward value*/

/* Shares and queues go here*/
Share<uint8_t> duty_cycle ("Motor duty cycle");
Share<bool> direction ("Motor direction");
Share<float> cube_angle ("Current cube angle off horizontal");

/* Instantiate a MPU9250 object with a ridiculous name */
MPU9250 MPUPU;

/* Instantiate MiniPID object and set Kp, Ki, Kd, and feed forward, respectively */
MiniPID PID(P,I,D,F);


/** @brief   Task which drives the motor. 
 *  @details This task runs directly after the PID outputs its updated PWM signal
 *           This task collects the values in the duty_cycle and direction shares 
 *           and outputs them to the motor. It then calls the IMU task to update 
 *           the current cube angle and begin running again. 
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void PWM (void* p_params)
{
(void)p_params;

  for(;;){
    /* pull the duty cycle from the share*/
    duty_cycle.get(mtr_duty);

    /* pull the direction from the share*/
    direction.get(mtr_dir);

    /* turn the motor according to the pulled values*/
    analogWrite(A3, mtr_duty);
    digitalWrite(A0, mtr_dir);

    /* Directly call IMU task*/
    void IMU();
}


}

/** @brief   Task which runs the PID controller
 *  @details This task runs directly after the IMU task. Using the angle in the share 
 *           as the current value, the PID controller generates an output that is put
 *           into the mtr_duty share. The direction is decided after and put into the
 *           direction share. Then the PWM task is called so the motor will receive
 *           the updated PWM signal.
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void PID ( void* p_params)
{
(void)p_params;

  for(;;) {

    /* get the current angle of the cube*/
    cube_angle.get(cube_ang);

    /* get the output of the PID and put it in the share*/
    mtr_duty = PID.getOutput(cube_ang, VERTICAL);
    duty_cycle.put(mtr_duty);

    error = cube_ang - VERTICAL;

      /* turn the motor in the correct direction to keep it balanced*/
      if (error < 1){
        direction.put(1);
      }
      else{
        direction.put(0);
      }

      /* Directly call PWM task*/
      void PWM();
}


/** @brief   Task which checks button status.
 *  @details This task runs directly after the PWM task. This task updates the IMU data
 *           and then collects all three angles of rotation. The correct angle is selected
 *           and placed into the cube_angle share. The PID task is run directly after the shares
 *           are updated. 
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void IMU(void* p_params)
{
(void)p_params;

  for(;;){

    /* collect the newest data*/
    MPUPU.update();

    /* calculate the updated angle data using these functions*/
    MPUPU.getRoll();
    MPUPU.getPitch();
    MPUPU.getYaw();

    /* not sure which angle we are looking for, but stuff it in the share and pass it on*/
    cube_angle.put(MPUPU.getYaw());

    /* Directly call PID task*/
    void PID();
}

/** @brief   Function that runs once to setup the balancing cube.
 *  @details This task runs once and does a number of things
 *           1) Verifies the MPU is connected and the PID is correctly set up. 
 *           2) Sets up motor direction and PWM pins
 *           3) Defines the tasks used to control the cube
 *          
 *           If setup is completed succesfully, 5 messages will be printed. 
 *           Make sure to read all messages before proceeding. 
 */
void setup() 
{
    /* Start the serial port, wait a short time, then say hello. Use the
    non-RTOS delay() function because the RTOS hasn't been started yet*/
    Serial.begin (115200);
    wire.begin();
    delay (2000);
    Serial << endl << endl << "ME507 Cube Balance Intializing..." << endl;

    /* Initialize MPU
     This will print a message displaying the MPU's memory address, verifying the MPU9250 is connected*/
    MPUPU.isConnectedMPU9250();
    /* This will store and print the MPU's bias*/
    MPUPU.calibrateAccelGyro();
    /* This will set up the MPU and print a message confirming connection*/
    MPUPU.setup(0x68);

    /* Initialize PID*/

    /* Note: Can also saturate the Integral controller so it is not jerky at the beginning of operation
    // when the error is large.*/
    
    /* Assign P,I,D, and F values to controller*/
    PID.setPID(P,I,D,F);
    /* Assign the setpoint where the cube is standing*/
    PID.setSetpoint(VERTICAL);
    /* Saturate output to the maximum duty cycle*/
    PID.setOutputLimits(MAX_DUTY);

  

    /* Initialize the motor and IMU pins*/
    /* Motor PWM pin*/
    pinMode(A3, OUTPUT);
    /* Motor direction pin*/
    pinMode(A0, OUTPUT);
  
    


/* Create the tasks as shown in the task diagram*/

    xTaskCreate (PWM,
                 "PWM Output to Motor",           // Name for printouts
                 1536,                            // Stack size
                 NULL,                            // Parameters for task fn.
                 3,                               // Priority
                 NULL);                           // Task handle
    xTaskCreate (PID,
                 "PID Outputs PWM",               // Name for printouts
                 1536,                            // Stack size
                 NULL,                            // Parameters for task fn.
                 2,                               // Priority
                 NULL);                           // Task handle
                        // Task handle
    xTaskCreate (IMU,
                 "Collects cube rotation",        // Name for printouts
                 1536,                            // Stack size
                 NULL,                            // Parameters for task fn.
                 1,                               // Priority
                 NULL);                           // Task handle           

    /* If using an STM32, we need to call the scheduler startup function now;
       if using an ESP32, it has already been called for us*/

    #if (defined STM32L4xx || defined STM32F4xx)
        vTaskStartScheduler ();
    #endif

    Serial << "Completed Initialization, please place the cube in the starting position." << endl;
}

void loop() 
{
  // put your main code here, to run repeatedly:
}