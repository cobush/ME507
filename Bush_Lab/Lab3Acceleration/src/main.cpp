// (c) Michael Schoeffler 2017, http://www.mschoeffler.de
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
#include <math.h>

Share<uint8_t> MPU_ADDR("Address of MPU6050");
Queue<int16_t> dadata(100, "Queued Data");

int16_t d;
int16_t ax;
int16_t ay;
int16_t az;
int16_t gx;
int16_t gy;
int16_t gz;
int16_t ax_offset;
int16_t ay_offset;
int16_t az_offset;
int16_t gx_offset;
int16_t gy_offset;
int16_t gz_offset;
double angle_x;
double angle_y;
double angle_z;
int16_t rotation_x = 0;
int16_t rotation_y = 0;
int16_t rotation_z = 0;
double calc;
uint8_t ADDR = 0;
void CollectData()
{

}
void setup()
{
// Start the serial port, wait a short time, then say hello. Use the
  // non-RTOS delay() function because the RTOS hasn't been started yet
  Serial.begin (115200);


  // Initialize interface between SDA and SCL pins on GY-251 and Nucleo
  pinMode(D14, INPUT);
  pinMode(D15, INPUT);

  // A0 pin on GY-251 is used to set the memory address of the MPU6050 
  pinMode(PC9, INPUT); 
  // Define address based on pin 
  if (digitalRead(PC9) == true)
  {
    MPU_ADDR.put(0x69);
  }
  else
  {
    MPU_ADDR.put(0x68);
  }


  MPU_ADDR.get(ADDR);

  MPU6050 Collector(ADDR);
  Wire.begin();
  Wire.beginTransmission(ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  

  Serial << endl << endl << "Hello, I am an RTOS demonstration." << endl;
  Serial << "I print angle displacements from horizontal. The x-y plane is a 'table' and the z axis is out of the 'table'" << endl << endl;


  // check I2C connection
  Serial << "Testing I2C connection..." << endl;

  if (Collector.testConnection() == 1)
  {
    Serial << "I2C communication status: CONNECTED"  << endl;
  }
  else
  {
    Serial << "I2C communication status: NOT CONNECTED" << endl << endl;
  }

  // // clear sensors of old data
  // Collector.resetSensors();


  // // Create a task which prints angle data
  // xTaskCreate (CollectData,
  //               "MPUE6050 Data Collector",       // Name for printouts
  //               1024,                            // Stack size
  //               NULL,               // Parameter(s) for task fn.
  //               1,                               // Priority
  //               NULL);                           // Task handle


  // // If using an STM32, we need to call the scheduler startup function now;
  // // if using an ESP32, it has already been called for us
  // #if (defined STM32L4xx || defined STM32F4xx)
  //     vTaskStartScheduler ();
  // #endif
}



void loop()
{
  MPU_ADDR.get(ADDR);
  MPU6050 Collector(ADDR);

  // Store all data from MPU6050 in the respective address defined
  Collector.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Collect the offsets for all measurements
  ax_offset = Collector.getXAccelOffset();
  ay_offset = Collector.getYAccelOffset();
  az_offset = Collector.getZAccelOffset();

  double axd = ax;
  double ayd = ay;
  double azd = az; 

  calc = ayd/sqrt((axd*axd) + (azd*azd));
  angle_x = atan(calc);
  angle_x = angle_x * 180/PI;
  Serial << "x-angle: " << angle_x << "  [degrees]   \r" ;
  calc = axd/sqrt((ayd*ayd) + (azd*azd));
  angle_y = atan(calc);
  angle_y = angle_y * 180/PI;
  calc = sqrt((axd*axd) + (ayd*ayd))/azd;
  angle_z = atan(calc);
  angle_z = angle_z * 180/PI;

  // DONT NEED THESE YET
  // gx_offset = Collector.getXGyroOffset();
  // gy_offset = Collector.getYGyroOffset();
  // gz_offset = Collector.getZGyroOffset();

  // calculate angles off each respective axis

  // Calibrate all raw data to read acceleration in g's and rotation in deg/s
  ax = (ax - ax_offset)/16384;
  ay = (ay - ay_offset)/16384;
  az = (az - az_offset)/16384;

  // DONT NEED THESE YET
  // gx = (gx - gx_offset)/131;
  // gy = (gy - gy_offset)/131;
  // gz = (gz - gz_offset)/131;



  // dadata.put(angle_x);
  // dadata.get(d);
  // double x = d;
  // Serial << "x-angle: " << x << " [degrees]      \r";


  // // Put the data in the queue in order: [angle_x, angle_y, angle_z, rotation_x, rotation_y, rotation_z, repeat..]
  // for (int i = 0; i < 3; i++)
  // {
  //   if (i==0)
  //   {

  //   }
  //   else if (i==1)
  //   {
  //     dadata.put(angle_y);
  //   }
  //   else if (i==2)
  //   {
  //     dadata.put(angle_z);
  //     i = -1;
    
  //   }
  //   // if (i==0)
  //   // {
      
  //   // }
  //   // if (i==0)
  //   // {
      
  //   // }
  //   // if (i==0)
  //   // {
      
  //   // }
  //   // if (i==0)
  //   // {
      
  //   // }
  //   else 
  //   {
  //     break;
  //   }
    
  // }

  // for (int i = 0; i < 3; i++)
  // {
  //   if (dadata.is_empty() == false)
  //   {
  //     dadata.get(d);
  //     if (i ==0)
  //     {
  //       Serial << "x-accel" << d << " m/s^2   \r" ;
  //     }
  //     else if (i == 1)
  //     {
  //       Serial << "y-accel" << d << " m/s^2   \r" ;
  //     }
  //     else if (i == 2)
  //     {
  //       Serial << "z-accel" << d << " m/s^2   \r" << endl;
  //       i = 0;
  //     }
    
  //   }
  //   else
  //   {
  //     Serial << "dadata Queue is empty     \r" << endl;
  //     break;
  //   }
    
  // }
  delay(2000);
}