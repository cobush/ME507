
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

// define general variables to be used when pulling data from shares
// bool one_bit;
// float float_val;
// uint16_t sixteen_bit;

// PID, PWM and IMU Variables
uint8_t mtr_duty;
bool mtr_dir;
float cube_ang;
// uint16_t imu;
// uint16_t generic_speed;
uint16_t error;
uint8_t MAX_DUTY = 255; // Maximum duty cycle output
double VERTICAL = 45; // constant value set to zero the gyro on the vertical axis
double P = 1; // Proportional constant
double I = 1; // Integral constant
double D = 1; // Derivative constant
double F = 0; // Feed forward value

// Shares and queues go here
Share<uint8_t> duty_cycle ("Motor duty cycle");
Share<bool> direction ("Motor direction");
// Share<uint16_t> gyro_data ("Current Cube Rotation");
// Share<uint16_t> motor_speed("Current Motor Speed");
// Share<uint16_t> cube_speed("Current Cube Speed");
Share<float> cube_angle ("Current cube angle off horizontal");
// Share<uint8_t> state ("Current state of the ");

// Instantiate a MPU9250 object with a ridiculous name 
MPU9250 MPUPU;
// Instantiate MiniPID object and set Kp, Ki, Kd, and feed forward, respectively
MiniPID PID(P,I,D,F);



void setup() 
{

  // put your setup code here, to run once:
  
    // Start the serial port, wait a short time, then say hello. Use the
    // non-RTOS delay() function because the RTOS hasn't been started yet
    Serial.begin (115200);
    delay (2000);
    Serial << endl << endl << "ME507 Cube Balance Intializing..." << endl;

    // Initialize MPU
    // This will print a message displaying the MPU's memory address, verifying the MPU9250 is connected
    MPUPU.isConnectedMPU9250();
    // This will store and print the MPU's bias
    MPUPU.calibrateAccelGyro();
    // This will set up the MPU and print a message confirming connection
    MPUPU.setup(0x68);

    // Initialize PID
    // We can saturate the result from the integral component if we want to eliminate jerky response at beginning
    // Assign P,I,D, and F values to controller
    PID.setPID(P,I,D,F);
    // Assign the setpoint where the cube is standing
    PID.setSetpoint(VERTICAL);
    // Saturate output to the maximum duty cycle
    PID.setOutputLimits(MAX_DUTY);

    // Initialize the motor and IMU
    // Motor PWM pin
    pinMode(A3, OUTPUT);
    // Motor direction pin
    pinMode(A0, OUTPUT);
    
    // // SDA pin
    // pinMode(PA15, INPUT);
    // // SDL pin
    // pinMode(PA14, INPUT);
    


// Create the tasks as shown in the task diagram

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

    // If using an STM32, we need to call the scheduler startup function now;
    // if using an ESP32, it has already been called for us

    #if (defined STM32L4xx || defined STM32F4xx)
        vTaskStartScheduler ();
    #endif

    Serial << "Completed Initialization, please place the cube in the starting position." << endl;
}

void PWM (void* p_params)
{
(void)p_params;

for(;;){
  // pull the duty cycle from the share
  duty_cycle.get(mtr_duty);

  // pull the direction from the share
  direction.get(mtr_dir);

  // turn the motor according to the pulled values
  analogWrite(A3, mtr_duty);
  digitalWrite(A0, mtr_dir);

  // update the PID output manually
  vTaskDelay(pdMS_TO_TICKS(5));
}


}
void PID ( void* p_params)
{
(void)p_params;

for(;;) {

  // get the current angle of the cube
  cube_angle.get(cube_ang);

  // get the output of the PID and put it in the share
  mtr_duty = PID.getOutput(cube_ang, VERTICAL);
  duty_cycle.put(mtr_duty);

  error = cube_ang - VERTICAL;

    // turn the motor in the correct direction to keep it balanced
    // NEEDS TO BE TESTED TO VERIFY CORRECT DIRECTIONS
    if (error < 1){
      direction.put(1);
    }
    else{
      direction.put(0);
    }

    // Update the PWM signal manually
    
    vTaskDelay(pdMS_TO_TICKS(5));
}



}
void IMU(void* p_params)
{
(void)p_params;

for(;;){

  // collect the newest data
  MPUPU.update();

  // calculate the updated angle data using these functions
  MPUPU.getRoll();
  MPUPU.getPitch();
  MPUPU.getYaw();

  // not sure which angle we are looking for, but stuff it in the share and pass it on
  cube_angle.put(MPUPU.getYaw());

  void PID();

vTaskDelay(pdMS_TO_TICKS(12));
}
}

void loop() 
{
  // put your main code here, to run repeatedly:
}