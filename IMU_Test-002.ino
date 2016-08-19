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
   Connect SCL to SCL 21 (Mega)
   Connect SDA to SDL 20 (Mega)
   Connect VDD to 5V DC
   Connect GROUND to common ground
*/

float angle_x = 0;
float angle_y = 0;
float angle_z = 0;

float dt = 0;
float timer = 0;

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
  // ************* LSM9DS0 Setup *************
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

  //************** ESC/Motor Setup ***************
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
  dt = (micros() - timer)/1000000;                    // calculate dt between each sample ****** CONSIDER "IF STATEMENT" IF EXECUTING INTERRUPT ****** 
  
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  float gyro_x = gyro.gyro.x;
  float gyro_y = gyro.gyro.y;
  float gyro_z = gyro.gyro.z;

  float accel_x = accel.acceleration.x;
  float accel_y = accel.acceleration.y;
  float accel_z = accel.acceleration.z;
  
  // PITCH ***************
  float accel_Y_angle = (atan2(accel_x, accel_z)+PI)*180/PI;
  float gyro_Y_angle = gyro_y/131;

  angle_y = (0.97)*(angle_y + gyro_Y_angle*dt) + (0.03)*(accel_Y_angle);

  // ROLL ****************
  float accel_X_angle = (atan2(accel_y, accel_z)+PI)*180/PI;
  float gyro_X_angle = gyro_x/131;

  angle_x = (0.97)*(angle_x + gyro_X_angle*dt) + (0.03)*(accel_X_angle);

  // YAW *****************
  float accel_Z_angle = (atan2(accel_y, accel_x)+PI)*180/PI;
  float gyro_Z_angle = gyro_z/131;

  angle_z = (0.97)*(angle_z + gyro_Z_angle*dt) + (0.03)*(accel_Z_angle);
  
  timer = micros();

  //Serial.print("Angle_Y: "); Serial.print(angle_y);
  //Serial.print("Angle X: "); Serial.print(angle_x);
  //Serial.print("Angle Z: "); Serial.print(angle_z);
  //Serial.print("Angle X: "); Serial.print(angle_x); Serial.print(" "); Serial.print("Angle Y: "); Serial.print(angle_y); Serial.print(" ");  Serial.print("Angle Z: "); Serial.print(angle_z);
  //Serial.print("\n");
 
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


