#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

// =========================================================================
// Begin Motor Configuration
#define MOTOR_PIN_A 2
#define MOTOR_PIN_B 3
#define MOTOR_PIN_C 4
#define MOTOR_PIN_D 5

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define STOP 0

int m_A = 0;
int m_B = 0;
int m_C = 0;
int m_D = 0;

Servo motor_A, motor_B, motor_C, motor_D;

void Motors_Initialize(void);
void Motors_Arm(void);
void Motors_Update(void);

// End Motor Configuration
//==========================================================================
// Begin IMU Configuration
/*
Adafruit LSM9DS0
Connections (For default I2C)
  ===========
  SCL to SCL 21 (Mega)
  SDA to SDL 20 (Mega)
  VDD to 5V DC
  GND to GND
*/

float angle_x = 0;
float angle_y = 0;
float angle_z = 0;

float dt = 0;
float timer = 0;

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000, **Assign a unique base ID for this sensor** 


void IMU_Initialize(void);
void IMU_Read(void);

// End IMU Configuration
//==========================================================================
// Begin Bluetooth Configuration
/*
Adafruit Bluefruit LE UART Friend
Connections (For Mega using HW UART)
  ============
  MOD to PIN 12
  CTS to GND
  TXO to PIN 18 (Serial1)
  RXI to PIN 19 (Serial1)
  VIN to 5V
  GND to GND
*/

#define BLUEFRUIT_HWSERIAL_NAME Serial1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];
void ReadSerial1(void);

void Bluetooth_Initialize(void);
void Bluetooth_Controller(void);

// End Bluetooth Configuration
// =========================================================================
// Begin PID Configuration
unsigned long run_time;
unsigned long current_time;
float previous_time;
int delta_t;
int sample_time = 100;                                      // 10 times per second

float setpoint_x = 180;
float setpoint_y = 180;
float error_x, error_y;
float input_x, input_y;
float integral_x, integral_y;
float d_input_x, d_input_y;
float output_x, output_y;
int output_MAX = 200;

float previous_input_x, previous_input_y;

float kP = 0.25;
float kI = 1;
float kD = 0.05;

float sample_time_seconds;
float kP_new = 0.25;
float kI_new = 1;
float kD_new = 0.05;

float ratio;
int new_sample_time;

bool in_auto = false;
int mode;

#define MANUAL 0
#define AUTOMATIC 1

bool new_auto = (mode == MANUAL);

void PID_Compute(void);
void PID_Set_Tuning(void);
void PID_Set_Sample_Time(void);
void PID_Set_Mode(void);
void PID_Initialize(void);

//End PID Configuration
//==========================================================================
void Motors_Initialize(){
  motor_A.attach(MOTOR_PIN_A);                                    // start PWM stream
  motor_B.attach(MOTOR_PIN_B);
  motor_C.attach(MOTOR_PIN_C);
  motor_D.attach(MOTOR_PIN_D);

  motor_A.writeMicroseconds(MIN_SIGNAL);                               
  motor_B.writeMicroseconds(MIN_SIGNAL);
  motor_C.writeMicroseconds(MIN_SIGNAL);
  motor_D.writeMicroseconds(MIN_SIGNAL);
}
// =========================================================================
void Motors_Arm(){
  motor_A.writeMicroseconds(MIN_SIGNAL);                               
  motor_B.writeMicroseconds(MIN_SIGNAL);
  motor_C.writeMicroseconds(MIN_SIGNAL);
  motor_D.writeMicroseconds(MIN_SIGNAL);
}
// =========================================================================
void Motors_Update(){
  m_A = m_A + output_x - output_y;                   // front left motor
  m_B = m_B - output_x - output_y;                   // front right motor
  m_C = m_C - output_x + output_y;                   // back right motor
  m_D = m_D + output_x + output_y;                   // back left motor

  if (m_A > MAX_SIGNAL)
    m_A = MAX_SIGNAL;
  else if (m_A < MIN_SIGNAL)
    m_A = MIN_SIGNAL;

  if (m_B > MAX_SIGNAL)
    m_B = MAX_SIGNAL;
  else if (m_B < MIN_SIGNAL)
    m_B = MIN_SIGNAL;

  if (m_C > MAX_SIGNAL)
    m_C = MAX_SIGNAL;
  else if (m_C < MIN_SIGNAL)
    m_C = MIN_SIGNAL;
  
  if (m_D > MAX_SIGNAL)
    m_D = MAX_SIGNAL;
  else if (m_D < MIN_SIGNAL)
    m_D = MIN_SIGNAL;

  //Serial.print(m_A); Serial.print(" ");  Serial.print(m_B); Serial.print(" ");   Serial.print(m_C); Serial.print(" ");   Serial.print(m_D);
  //Serial.print("\n");

  motor_A.writeMicroseconds(m_A);                               
  motor_B.writeMicroseconds(m_B);
  motor_C.writeMicroseconds(m_C);
  motor_D.writeMicroseconds(m_D);
}
//==========================================================================
void IMU_Initialize(){
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
}
// =========================================================================
void IMU_Read(){
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
// =========================================================================
void Bluetooth_Initialize(){
  while (!Serial);  // required for Flora & Micro
  //delay(500);

  Serial1.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  //while (! ble.isConnected()) {
     // delay(500);
 // }

  Serial.println(F("******************************"));
  
  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

}
// =========================================================================
void Bluetooth_Control(){
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;
  
  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
      if (buttnum == 1)
        m_A += 25;
      else if (buttnum == 2)
        m_B += 25;
      else if (buttnum == 4)
        m_C += 25;
      else if (buttnum == 3)
        m_D += 25;
      else if (buttnum == 5){
        m_A += 50;
        m_B += 50;
        m_C += 50;
        m_D += 50;
      }
      else if (buttnum == 6){
        m_A -= 50;
        m_B -= 50;
        m_C -= 50;
        m_D -= 50;
      }
      else if (buttnum == 8){
        mode = AUTOMATIC;
      }
      else if (buttnum == 7){
        mode = MANUAL;
        m_A = STOP;
        m_B = STOP;
        m_C = STOP;
        m_D = STOP;
      }
    }
    else{
      Serial.println(" released");
    }
  }
}
// =========================================================================
void PID_Compute(){
  if (!in_auto){
    setpoint_x = angle_x;
    setpoint_y = angle_y; 
    Serial.print(setpoint_x); Serial.print(" ");  Serial.print(setpoint_y);
    Serial.print("\n");
    return; 
  }
  current_time = millis();
  delta_t = current_time - previous_time;

  input_x = angle_x;
  input_y = angle_y;
  
  if (delta_t >= sample_time){                                    // compute once the sample time is reached. establishes regular interval for computation
    // ROLL ********************
    error_x = setpoint_x - input_x;                               // compute error. + when left side dips, - when right side dips
    integral_x += (kI*error_x);                                   // compute integral term with kI so changes in kI only affect future computations
    if (integral_x > output_MAX)                                  // limit I_term_x to values within output range
      integral_x = output_MAX;
    else if (integral_x < output_MAX*-1)                          // allows for bounded negative output 
      integral_x = output_MAX*-1; 
    d_input_x = (input_x - previous_input_x);                     // compute derivative term. eliminates derivative kick by using input terms
    
    output_x = (kP*error_x + integral_x - kD*d_input_x);          // compute PID output
    if (output_x > output_MAX)                                    // limit output_x to values within output range
      output_x = output_MAX;
    else if (output_x < output_MAX*-1)                            // allows for bounded negative output
      output_x = output_MAX*-1;

    // PITCH ********************
    error_y = setpoint_y - input_y;                               // compute error. + when left side dips, - when right side dips
    integral_y += (kI*error_y);                                   // compute integral term with kI so changes in kI only affect future computations
    if (integral_y > output_MAX)                                  // limit I_term_y to values within output range
      integral_y = output_MAX;
    else if (integral_y < output_MAX*-1)                          // allows for bounded negative output 
      integral_y = output_MAX*-1; 
    d_input_y = (input_y - previous_input_y);                     // compute derivative term. eliminates derivative kick by using input terms
    
    output_y = (kP*error_y + integral_y - kD*d_input_y);          // compute PID output
    if (output_y > output_MAX)                                    // limit output_y to values within output range
      output_y = output_MAX;
    else if (output_y < output_MAX*-1)                            // allows for bounded negative output
      output_y = output_MAX*-1;

    previous_input_x = input_x;                                   // recalculate previous input_x
    previous_input_y = input_y;                                   // recalculate previous input_y
    previous_time = current_time;
  }
}
// =========================================================================
void PID_Set_Tuning(){
  sample_time_seconds = ((float)sample_time)/1000;
  kP = kP_new;
  kI = kI_new*sample_time_seconds;
  kD = kD_new/sample_time_seconds;
}
// =========================================================================
void PID_Set_Sample_Time(){
  if (new_sample_time > 0){
    ratio = (float)new_sample_time/sample_time;
    kI *= ratio;
    kD /= ratio;
    sample_time = (unsigned long)new_sample_time;
  }
}
// =========================================================================
void PID_Set_Mode(){
  new_auto = mode;
  if(new_auto && !in_auto){                                       // just switched from manual to automatic
    PID_Initialize();
  }
  if (in_auto){
    
  }
  in_auto = new_auto;
}
// =========================================================================
void PID_Initialize(){
  // ROLL *********************
  previous_input_x = input_x;
  integral_x = output_x;
  if (integral_x > output_MAX)
    integral_x = output_MAX;
  else if (integral_x < output_MAX*-1)
    integral_x = output_MAX*-1;

  // PITCH *********************
  previous_input_y = input_y;
  integral_y = output_y;
  if (integral_y > output_MAX)
    integral_y = output_MAX;
  else if (integral_y < output_MAX*-1)
    integral_y = output_MAX*-1;
}

// =========================================================================
void setup() {
  Serial.begin(9600);

  Motors_Initialize();
  IMU_Initialize();
  Bluetooth_Initialize();
  Motors_Arm(); 
}

void loop(){
  IMU_Read();
  Bluetooth_Control();
  PID_Compute();
  PID_Set_Tuning();
  PID_Set_Sample_Time();
  PID_Set_Mode();
  PID_Initialize();
  Motors_Update();
}
