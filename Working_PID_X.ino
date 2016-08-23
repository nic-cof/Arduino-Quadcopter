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

//Servo motor_A, motor_B, motor_C, motor_D;

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1100
#define STOP 0

int arm = 1000;
/*
int ESC_speed_A = 0;
int ESC_speed_B = 0;
int ESC_speed_C = 0;
int ESC_speed_D = 0;
*/
int m_A = 0;
int m_B = 0;
int m_C = 0;
int m_D = 0;

int motor_A_throttle = 0;
int motor_B_throttle = 0;
int motor_C_throttle = 0;
int motor_D_throttle = 0;

Servo motor_A, motor_B, motor_C, motor_D;

void ControlMotors(void);

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

void ReadSensor(void);

// End IMU Configuration


// ************************************************************************
// Begin PID Configuration
unsigned long current_time;
double previous_time;
int delta_t;
int sample_time = 100;                                      // 100 times per second

double setpoint_x = 180;
double error_x;
double input_x;
double integral_x;
double d_input_x;

double output_x;
int output_MAX = 900;
double MAX;
double MIN;

double previous_input_x;

double kP = 1;
double kI = 1;
double kD = 1;

double sample_time_seconds;
double kP_new = 1;
double kI_new = 1;
double kD_new = 1;

double ratio;
int new_sample_time;

void ComputePID(void);
void SetTuning(void);
void SetSampleTime(void);

// End PID Configuration


// ************************************************************************
void setup() {
  Serial.begin(9600);
  
  // ************* LSM9DS0 Setup *************
  #ifndef ESP8266
  while (!Serial);                      // will pause Zero, Leonardo, etc until serial console opens
  #endif
  
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");

  if(!lsm.begin())                      // Initialize the sensor
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  //configureSensor();
  Serial.println("Sensors Configured");

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

  //************** ESC/Motor Setup *************** 
  motor_A.attach(MOTOR_PIN_A);                                  // start PWM stream
  motor_B.attach(MOTOR_PIN_B);
  motor_C.attach(MOTOR_PIN_C);
  motor_D.attach(MOTOR_PIN_D);

  motor_A.writeMicroseconds(arm);                               // arm ESCs
  motor_B.writeMicroseconds(arm);
  motor_C.writeMicroseconds(arm);
  motor_D.writeMicroseconds(arm);
}


// *****************************************************************************
void ReadSensor(){
  dt = (micros() - timer)/1000000;                              // calculate dt between each sample ****** CONSIDER "IF STATEMENT" IF EXECUTING INTERRUPT ****** 
  
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
}


// *****************************************************************************
void ComputePID(){                                              // compute PID for positive x direction
  current_time = millis();
  delta_t = current_time - previous_time;

  input_x = angle_x;
  
  if (delta_t >= sample_time){
    // ROLL ********************
    // compute working variables
    error_x = setpoint_x - input_x;                               // + when left side dips, - when right side dips
    integral_x += kI*error_x;                                     // 
    if (integral_x > output_MAX)                                  // limit I_term_x to values within output range
      integral_x = output_MAX;
    else if (integral_x < output_MAX*-1)                          // could be problematic here because we need negative numbers included 
      integral_x = output_MAX*-1;                                 // e.g. if the PID returns a negative number then the opposite of a positive number must happen (split PID for each side of quad?)
    d_input_x = input_x - previous_input_x;
    
    // compute PID output
    output_x = (kP*error_x + integral_x - kD*d_input_x);
    if (output_x > output_MAX)
      output_x = output_MAX;
    else if (output_x < output_MAX*-1)
      output_x = output_MAX*-1;

    previous_input_x = input_x;
    previous_time = current_time;
  }
}


// *****************************************************************************
void SetTuning(){
  sample_time_seconds = ((double)sample_time)/1000;
  kP = kP_new;
  kI = kI_new*sample_time_seconds;
  kD = kD_new/sample_time_seconds;
}


// *****************************************************************************
void SetSampleTime(){
  if (new_sample_time > 0){
    ratio = (double)new_sample_time/sample_time;
    kI *= ratio;
    kD /= ratio;
    sample_time = (unsigned long)new_sample_time;
  }
}


// *****************************************************************************
/*void SetOutputLimits(){
  if (MIN > MAX)
    return;
  output_MIN = MIN;
  output_MAX = MAX;

  if (output_x > output_MAX)
    output_x = output_MAX;
  else if (output_x < output_MIN)
    output_x = output_MIN;

  if (integral_x > output_MAX)
    integral_x = output_MAX;
  else if (integral_x < output_MIN)
    integral_x = output_MIN;
}
*/

// *****************************************************************************
void UpdateMotors(){
  m_A = motor_A_throttle + output_x;
  m_B = motor_B_throttle - output_x;
  m_C = motor_C_throttle - output_x;
  m_D = motor_D_throttle + output_x;

  Serial.print("Left: "); Serial.print(m_A); Serial.print(" "); Serial.print("Right: "); Serial.print(m_B);
  Serial.print("\n");

}


// *****************************************************************************
void ControlMotors(){
  motor_A.writeMicroseconds(m_A);                  
  motor_B.writeMicroseconds(m_B);
  motor_C.writeMicroseconds(m_C);
  motor_D.writeMicroseconds(m_D);
}


// *****************************************************************************
void loop(){
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
    ReadSensor();
    ComputePID();
    SetTuning();
    SetSampleTime();
    UpdateMotors();
    ControlMotors();
    }
}

