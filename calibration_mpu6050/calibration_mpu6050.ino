#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// ======================== MPU6050 VARIABLES ========================
// Status flag for DMP (Digital Motion Processor) initialization
uint8_t dmpStatus;

MPU6050 mpu;   // MPU6050 sensor instance

// ======================== ARDUINO SETUP ========================
void setup() {
  // --- I2C Configuration ---
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();              // Initialize I2C
    Wire.setClock(400000);     // Use fast mode (400kHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // --- Serial Communication ---
  Serial.begin(115200);        // Open serial monitor at 115200 baud
  while (!Serial);             // Wait until serial is ready (for some boards)

  Serial.println("Initializing I2C devices...");
  mpu.initialize();            // Initialize MPU6050

  // --- Device Connection Test ---
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ?
                 "MPU6050 Connection Successful" :
                 "MPU6050 Connection Failed");

  // --- Wait for user input before continuing ---
  Serial.println("\nSend any character to begin DMP programming and demo: ");
  while (!Serial.available());

  // --- DMP Initialization ---
  Serial.println("Initializing DMP...");
  dmpStatus = mpu.dmpInitialize();
  Serial.println(dmpStatus);   // Print DMP status (0 = success)

  // --- Sensor Calibration if DMP init succeeded ---
  if (dmpStatus == 0) {
    mpu.CalibrateAccel(6);     // Accelerometer calibration (6 samples)
    mpu.CalibrateGyro(6);      // Gyroscope calibration (6 samples)
    mpu.PrintActiveOffsets();  // Print calibrated offset values
  }
}

// ======================== ARDUINO LOOP ========================
// Loop is intentionally empty. This script focuses on initialization
// and calibration of the MPU6050. Data acquisition and processing
// can be added here after successful DMP setup.
void loop() {
}