#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>

// ======================== PID CONFIGURATION ========================
// Define PID limits and tuning constants for balancing and yaw control.
// These values are tuned experimentally and may need adjustments
// depending on robot weight distribution, motor torque, and wheel friction.
#define PID_MIN_LIMIT -255
#define PID_MAX_LIMIT  255
#define PID_SAMPLE_TIME 10    // PID sample time in ms
#define MIN_SPEED 0           // Minimum motor speed offset to overcome deadband

// Pitch stabilization gains (robot balance)
#define PITCH_KP 10
#define PITCH_KI 80
#define PITCH_KD 0.8

// Yaw stabilization gains (rotation correction)
#define YAW_KP 0.5
#define YAW_KI 0.5
#define YAW_KD 0

// ======================== PID VARIABLES ========================
double desiredPitchAngle = 0;   // Target pitch (upright position = 0°)
double pitchAngle = 0;          // Measured pitch angle
double pitchOutput = 0;         // PID controller output for pitch

double desiredyawRate = 0;      // Target yaw rate (0 = straight motion)
double yawRate = 0;             // Measured yaw rate from gyro
double yawOutput = 0;           // PID controller output for yaw

// ======================== MPU6050 VARIABLES ========================
uint8_t dmpStatus;              // Status of DMP initialization
uint8_t fifoBuffer[64];         // FIFO buffer for DMP data

float YawPitchRoll[3];          // Yaw, Pitch, Roll values
Quaternion q;                   // Quaternion data
VectorFloat gravity;            // Gravity vector
VectorInt16 gy;                 // Gyroscope data

MPU6050 mpu;                    // MPU6050 instance

// Initialize PID controllers for pitch and yaw
PID pitchPID(&pitchAngle, &pitchOutput, &desiredPitchAngle,
             PITCH_KP, PITCH_KI, PITCH_KD, DIRECT);

PID yawPID(&yawRate, &yawOutput, &desiredyawRate,
           YAW_KP, YAW_KI, YAW_KD, DIRECT);

// ======================== MOTOR DRIVER PINS ========================
// Dual motor driver (H-bridge) connections
int ENA = 10;   // Right motor speed (PWM)
int IN1 = 8;    // Right motor direction
int IN2 = 7;    
int IN3 = 6;    // Left motor direction
int IN4 = 5;
int ENB = 9;    // Left motor speed (PWM)

// ======================== SETUP FUNCTIONS ========================

// Configure motor pins
void setupMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Stop motors initially
  rotateMotor(0, 0);
}

// Configure MPU6050 IMU
void setupMpu() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // Use fast I2C mode (400kHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  mpu.initialize();
  dmpStatus = mpu.dmpInitialize();

  // Sensor offsets (calibrated manually or via calibration script)
  mpu.setXAccelOffset(-1798);
  mpu.setYAccelOffset(263);
  mpu.setZAccelOffset(1124);
  mpu.setXGyroOffset(-261);
  mpu.setYGyroOffset(26);
  mpu.setZGyroOffset(32);

  // Enable DMP if initialization is successful
  if (dmpStatus == 0) {
    mpu.setDMPEnabled(true);
  } else {
    Serial.println("DMP Initialization Failed");
  }
}

// Configure PID controllers
void setupPid() {
  // Pitch PID
  pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(PID_SAMPLE_TIME);

  // Yaw PID
  yawPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetSampleTime(PID_SAMPLE_TIME);
}

// ======================== ARDUINO CORE ========================
void setup() {
  Wire.begin();
  setupMotors();
  setupMpu();
  setupPid();
}

void loop() {
  // If DMP failed, stop execution
  if (dmpStatus != 0) return;

  // Check if new IMU packet is available
  uint8_t isPacketPresent = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);

  if (isPacketPresent == true) {
    // Extract orientation (quaternion → yaw, pitch, roll)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(YawPitchRoll, &q, &gravity);

    pitchAngle = YawPitchRoll[1] * 180 / M_PI;  // Convert radians → degrees

    // Extract gyro data for yaw rate control
    mpu.dmpGetGyro(&gy, fifoBuffer);
    yawRate = gy.z;
  }

  // Run PID controllers
  pitchPID.Compute();
  yawPID.Compute();

  // Adjust motor speeds (pitch for balance, yaw for steering stability)
  rotateMotor(pitchOutput - yawOutput, pitchOutput + yawOutput);
}

// ======================== MOTOR CONTROL ========================
// Control both motors based on PID outputs
void rotateMotor(int leftMotorSpeed, int rightMotorSpeed) {
  // --- Right Motor Direction ---
  if (rightMotorSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (rightMotorSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // --- Left Motor Direction ---
  if (leftMotorSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (leftMotorSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // --- Speed Control with Deadband Compensation ---
  rightMotorSpeed = abs(rightMotorSpeed) + MIN_SPEED;
  leftMotorSpeed  = abs(leftMotorSpeed)  + MIN_SPEED;
  
  rightMotorSpeed = constrain(rightMotorSpeed, MIN_SPEED, 255);
  leftMotorSpeed  = constrain(leftMotorSpeed, MIN_SPEED, 255);

  analogWrite(ENA, rightMotorSpeed);
  analogWrite(ENB, leftMotorSpeed);
}