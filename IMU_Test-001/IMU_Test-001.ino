#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

// **********************************************************************
// Begin IMU Configuration
/*
Connections (For default I2C)
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 5V DC
   Connect GROUND to common ground
*/

float angle_x = 0;
float angle_y = 0;
float angle_z = 0;

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000, **Assign a unique base ID for this sensor**

void configureSensor(void);
void readSensor(void);

// End IMU Configuration
// ************************************************************************
// Begin Motor Configuration

int throttle = 0;                        // Set throttle equal to zero
Servo motor_A, motor_B, motor_C, motor_D;

// End Motor Configuration
// ************************************************************************

void setup() {
  #ifndef ESP8266
  while (!Serial);                      // will pause Zero, Leonardo, etc until serial console opens
  #endif
  Serial.begin(9600);
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");

  /* Initialize the sensor */
  if(!lsm.begin())                      // Initialize the sensor
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  configureSensor();
  Serial.println("Sensors Configured");
  
  motor_A.attach(2);
  motor_B.attach(3);
  motor_C.attach(4);
  motor_D.attach(5);
}

// *********************************************************************

void configureSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}


// *****************************************************************************

void readSensor()
{
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  
  float gyro_x = gyro.gyro.x;
  float gyro_y = gyro.gyro.y;
  float gyro_z = gyro.gyro.z;

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  double  accel_x = (atan2(ay,az)+PI)*RAD_TO_DEG;
  double  accel_y = (atan2(ax,az)+PI)*RAD_TO_DEG;
  double  accel_z = (atan2(ay,ax)+PI)*RAD_TO_DEG;
  
  angle_x = 0.94*(angle_x + gyro_x*0.01) + 0.06*(accel_x);
  angle_y = 0.94*(angle_y + gyro_y*0.01) + 0.06*(accel_y);
  angle_z = 0.94*(angle_z + gyro_z*0.01) + 0.06*(accel_z);
  
  Serial.print("Accel X: "); Serial.print(angle_x); Serial.print(" "); Serial.print("Accel Y: "); Serial.print(angle_y); Serial.print(" ");  Serial.print("Accel Z: "); Serial.print(angle_z);
  Serial.print("\n");

  delay(250);
 
  //Serial.print("Sensor test");
  //Serial.print("\n");
  
}


// *****************************************************************************

void loop() {
  // First connect ESC without arming. Then open Serial and follow instructions.

  motor_A.writeMicroseconds(throttle);   // This Value is controlled by user input from Serial Port
  motor_B.writeMicroseconds(throttle);
  motor_C.writeMicroseconds(throttle);
  motor_D.writeMicroseconds(throttle);
  
  if(Serial.available())
    throttle = Serial.parseInt();        // Parse Int from Serial

  else {
    readSensor();
  }
}


