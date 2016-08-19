#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>


// ************************************************************************
// Begin Motor Configuration
#define MOTOR_PIN_A 2
#define MOTOR_PIN_B 3
#define MOTOR_PIN_C 4
#define MOTOR_PIN_D 5

Servo motor_A, motor_B, motor_C, motor_D;

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1100
#define STOP 0

int ESC_speed_A = 0;
int ESC_speed_B = 0;
int ESC_speed_C = 0;
int ESC_speed_D = 0;

int motor_A_throttle = 0;
int motor_B_throttle = 0;
int motor_C_throttle = 0;
int motor_D_throttle = 0;

void controlMotors(void);

// End Motor Configuration


// ************************************************************************
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
// Begin PID Configuration
float error_x = 0;
float error_y = 0;
float error_z = 0;

float previous_error_x = 0;
float previous_error_y = 0;
float previous_error_z = 0;

int Px = 0;
int Py = 0;
int Pz = 0;

int Ix = 0;
int Iy = 0;
int Iz = 0;

int Dx = 0;
int Dy = 0;
int Dz = 0;

float previous_Ix = 0;
float previous_Iy = 0;
float previous_Iz = 0;

float Kp = 0;
float Ki = 0;
float Kd = 0;

float PID_x = 0;
float PID_y = 0;
float PID_z = 0;

// End PID Configuration


// ************************************************************************
void setup() {
  Serial.begin(9600);
  
  // ************* LSM9DS0 Setup *************
  #ifndef ESP8266
  while (!Serial);                      // will pause Zero, Leonardo, etc until serial console opens
  #endif
  
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
  motor_A.attach(MOTOR_PIN_A);
  motor_B.attach(MOTOR_PIN_B);
  motor_C.attach(MOTOR_PIN_C);
  motor_D.attach(MOTOR_PIN_D);
/*
  Serial.println("Outputting Maximum Power...");
  motor_A.writeMicroseconds(MAX_SIGNAL);
  motor_B.writeMicroseconds(MAX_SIGNAL);
  motor_C.writeMicroseconds(MAX_SIGNAL);
  motor_D.writeMicroseconds(MAX_SIGNAL);
  
  Serial.println("Turn on Power source, wait 2 seconds and press any key...");
  while(!Serial.available());
  Serial.read();

  Serial.println("Outputting Minimum Power...");
  motor_A.writeMicroseconds(MIN_SIGNAL);
  motor_B.writeMicroseconds(MIN_SIGNAL);
  motor_C.writeMicroseconds(MIN_SIGNAL);
  motor_D.writeMicroseconds(MIN_SIGNAL);
  
  Serial.println("Motor Calibration Complete.");
  delay(2000);
  */
}


// *********************************************************************
void controlMotors() {
    // Consirains Motor Speeds   
    if (ESC_speed_A > MAX_SIGNAL)
      ESC_speed_A = MAX_SIGNAL;
    if (ESC_speed_A < MIN_SIGNAL)
     ESC_speed_A = MIN_SIGNAL;

    if (ESC_speed_B > MAX_SIGNAL)
      ESC_speed_B = MAX_SIGNAL;
    if (ESC_speed_B < MIN_SIGNAL)
     ESC_speed_B = MIN_SIGNAL;

     if (ESC_speed_C > MAX_SIGNAL)
      ESC_speed_C = MAX_SIGNAL;
    if (ESC_speed_C < MIN_SIGNAL)
     ESC_speed_C = MIN_SIGNAL;

     if (ESC_speed_D > MAX_SIGNAL)
      ESC_speed_D = MAX_SIGNAL;
    if (ESC_speed_D < MIN_SIGNAL)
     ESC_speed_D = MIN_SIGNAL;

    motor_A.writeMicroseconds(ESC_speed_A);
    motor_B.writeMicroseconds(ESC_speed_B);
    motor_C.writeMicroseconds(ESC_speed_C);
    motor_D.writeMicroseconds(ESC_speed_D); 
}


// ******************************************************************************
void configureSensor(){
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
void readSensor(){
  dt = (micros() - timer)/1000000;                    // calculate dt between each sample ****** CONSIDER "IF STATEMENT" IF EXECUTING INTERRUPT ****** 
  
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  float gyro_x = gyro.gyro.x;
  float gyro_y = gyro.gyro.y;
  float gyro_z = gyro.gyro.z;

  float accel_x = accel.acceleration.x;
  float accel_y = accel.acceleration.y;
  float accel_z = accel.acceleration.z;

  // ROLL ****************
  float accel_X_angle = (atan2(accel_y, accel_z)+PI)*180/PI;
  float gyro_X_angle = gyro_x/131;

  angle_x = (0.97)*(angle_x + gyro_X_angle*dt) + (0.03)*(accel_X_angle);

  // PITCH ***************
  float accel_Y_angle = (atan2(accel_x, accel_z)+PI)*180/PI;
  float gyro_Y_angle = gyro_y/131;

  angle_y = (0.97)*(angle_y + gyro_Y_angle*dt) + (0.03)*(accel_Y_angle);

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
void calculateError(){
  if(angle_x != 180)
    error_x = angle_x - 180;
  else
    error_x = 0;
  //Serial.print("Error_X: "); Serial.print(error_x);
  //Serial.print("\n");
    
  if(angle_y != 180)
    error_y = angle_y - 180;
  else
    error_y = 0;
  //Serial.print("Error_Y: "); Serial.print(error_y);
  //Serial.print("\n");

  /*
  if(angle_z != 180)
    error_z = angle_z - 180;
  else
    error_z = 0;
    //Serial.print("Error_Y: "); Serial.print(error_y);
    //Serial.print("\n");
  */
}


// *****************************************************************************
void calculatePID(){
    Px = error_x;
    Ix = previous_Ix + error_x;
    Dx = error_x - previous_error_x;
    PID_x = Kp*Px + Ki*Ix + Kd*Dx;

    previous_Ix = Ix;
    previous_error_x = error_x;
    
    Py = error_y;
    Iy = previous_Iy + error_y;
    Dy = error_y - previous_error_y;
    PID_y = Kp*Py + Ki*Iy + Kd*Dy;

    previous_Iy = Iy;
    previous_error_y = error_y;
/*
    Pz = error_z;
    Iz = previous_Iz + error_z;
    Dz = error_z - previous_error_z;
    PID_z = Kp*Pz + Ki*Iz + Kd*Dz;
    
    previous_Iz = Iz;
    previous_error_z = error_z;
*/
}


// *****************************************************************************
void loop() {
  motor_A.writeMicroseconds(motor_A_throttle);   // This Value is controlled by user input from Serial Port
  motor_B.writeMicroseconds(motor_B_throttle);
  motor_C.writeMicroseconds(motor_C_throttle);
  motor_D.writeMicroseconds(motor_D_throttle);
  
  if (Serial.available() > 0) { 
    Serial.println("Input throttle.");
    char ch = Serial.read();
      if (ch == 'o'){
        motor_A_throttle = MIN_SIGNAL;
        motor_B_throttle = MIN_SIGNAL;
        motor_C_throttle = MIN_SIGNAL;
        motor_D_throttle = MIN_SIGNAL;
        }
      else if (ch == 'u'){
        motor_A_throttle += 100;
        motor_B_throttle += 100;
        motor_C_throttle += 100;
        motor_D_throttle += 100;
        }  
      else if (ch == 'd'){
        motor_A_throttle -= 100;
        motor_B_throttle -= 100;
        motor_C_throttle -= 100;
        motor_D_throttle -= 100;
        } 
      else if (ch == 's'){
        motor_A_throttle = STOP;
        motor_B_throttle = STOP;
        motor_C_throttle = STOP;
        motor_D_throttle = STOP;
        }
  }

  else {
    readSensor();
    controlMotors();
    calculateError();
    calculatePID();
  }
}


