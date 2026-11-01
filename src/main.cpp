#include <Arduino.h> // Required for PlatformIO/pioarduino projects
#include <AccelStepper.h> // Include the AccelStepper library for motor control
#include <math.h> // Include the math library for calculations
#include <Wire.h> // Include the Wire library for I2C communication
#include <ESP32Servo.h>
#include <DHEquations.h>
#include <array>
#include <InverseKinematics.h>
#include <AS5600.h>

#define baseOffset 115.60000 // Base offset for the Z-axis (adjust if needed) (mm)
#define shoulderOffset 109.50000 // Offset of the shoulder segment (adjust if needed) (mm)
#define shoulderLength 175.00000 // Length of the shoulder segment (adjust if needed) (mm)
#define primaryArmLength 175.00000 // Length of the arm segment (adjust if needed) (mm)
#define secondaryArmLength 208.00000 // Length of the secondary arm segment (adjust if needed) (mm)
#define differentialOffset 94.88388 // Offset of the first differential segment (adjust if needed) (mm)

#define baseGearRatio 5.0 // Gear ratio for the base segment (adjust if needed)
#define shoulderGearRatio 10.0 // Gear ratio for the shoulder segment (adjust if needed)
#define primaryArmGearRatio 10.0 // Gear ratio for the primary arm segment (adjust if needed)
#define wristGearRatio 10.0 // Gear ratio for the secondary arm segment (adjust if needed)
#define differentialGearRatio 10.0 // Gear ratio for the first differential segment (adjust if needed)
#define zWristGearRatio 3.0 // Gear ratio for the differential segment (adjust if needed)

#define baseAngularOffset 0.0 // Angular offset for the base segment (radians) // Adjust if needed
#define shoulderAngularOffset 0.0 // Angular offset for the shoulder segment (radians) // Adjust if needed
#define primaryArmAngularOffset 0.0 // Angular offset for the primary arm segment (radians) // Adjust if needed
#define wristAngularOffset 0.0 // Angular offset for the wrist segment (radians) // Adjust if needed
#define differential1AngularOffset 0.0 // Angular offset for the first differential segment (radians) // Adjust if needed
#define zWristAngularOffset 0.0 // Angular offset for the second differential segment (radians) // Adjust if needed

#define baseP 0.0 // Proportional gain for the base segment (adjust if needed)
#define shoulderP 0.0 // Proportional gain for the shoulder segment (adjust if needed)
#define primaryArmP 0.0 // Proportional gain for the primary arm segment (adjust if needed)
#define wristP 0.0 // Proportional gain for the wrist segment (adjust if needed)
#define differential1P 0.0 // Proportional gain for the first differential segment (adjust if needed)
#define zWristP 0.0 // Proportional gain for the second differential segment (adjust if needed)

#define baseI 0.0 // Integral gain for the base segment (adjust if needed)
#define shoulderI 0.0 // Integral gain for the shoulder segment (adjust if needed)
#define primaryArmI 0.0 // Integral gain for the primary arm segment (adjust if needed)
#define wristI 0.0 // Integral gain for the wrist segment (adjust if needed)
#define differential1I 0.0 // Integral gain for the first differential segment (adjust if needed)
#define zWristI 0.0 // Integral gain for the second differential segment (adjust if needed)

#define baseD 0.0 // Derivative gain for the base segment (adjust if needed)
#define shoulderD 0.0 // Derivative gain for the shoulder segment (adjust if needed )
#define primaryArmD 0.0 // Derivative gain for the primary arm segment (adjust if needed)
#define wristD 0.0 // Derivative gain for the wrist segment (adjust if needed)
#define differential1D 0.0 // Derivative gain for the first differential segment (adjust if needed)
#define zWristD 0.0 // Derivative gain for the second differential segment (adjust if needed)

#define TCAADDR 0x70 //I2C address for the TCA9548A I2C multiplexer

#define angularOffsets {baseAngularOffset, shoulderAngularOffset, primaryArmAngularOffset, wristAngularOffset, differential1AngularOffset, zWristAngularOffset} // Array of angular offsets for each segment

AccelStepper baseStepper(AccelStepper::DRIVER, 2, 3); // Create a stepper object for the base segament (pins 2 and 3)
AccelStepper shoulderStepper(AccelStepper::DRIVER, 4, 5); // Create a stepper object for the shoulder segment (pins 4 and 5)
AccelStepper primaryArmStepper(AccelStepper::DRIVER, 6, 7); // Create a stepper object for the primary arm segment (pins 6 and 7)
AccelStepper wristRotationStepper(AccelStepper::DRIVER, 8, 9); // Create a stepper object for the wrist rotation segment (pins 8 and 9)
AccelStepper differential1Stepper(AccelStepper::DRIVER, 10, 11); // Create a stepper object for the first differential segment (pins 10 and 11)
AccelStepper zWristRotationStepper(AccelStepper::DRIVER, 12, 13); // Create a stepper object for the second differential
Servo differentialWristServo; // Create a servo object for the differential wrist segment

std::array<double, 3> getPositionVector(Mat4 endEffectorTransform) {
  std::array<double, 3> positionVector = {
    endEffectorTransform[0][3],
    endEffectorTransform[1][3],
    endEffectorTransform[2][3]
  };
  return positionVector;
}

std::array<double, 3> getRollPitchYaw(Mat4 endEffectorTransform) {
  std::array<double, 3> rollPitchYaw = {0.0, 0.0, 0.0}; //Roll, pitch, and yaw angles for the end effector
  // Convert the initial orientation vector to roll, pitch, and yaw angles
  rollPitchYaw[0] = atan2(endEffectorTransform[2][1], endEffectorTransform[2][2]); // Roll
  rollPitchYaw[1] = atan2(-endEffectorTransform[2][0], sqrt(pow(endEffectorTransform[0][0], 2) + pow(endEffectorTransform[1][0], 2))); // Pitch
  rollPitchYaw[2] = atan2(endEffectorTransform[1][0], endEffectorTransform[0][0]); // Yaw
  return rollPitchYaw;
} 

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i); // Shift a bit to select channel 0-7
  Wire.endTransmission();
}

std::array<double, 6> readAngles() { 
  std::array<double, 6> angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Initialize an array to hold the angles of the segments
  for (int i = 0; i < 6; i++) {

    tcaselect(i); // Select the appropriate channel on the I2C multiplexer
    AS5600 encoder; // Create an instance of the AS5600 encoder class
    encoder.begin(); // Initialize the encoder
    angles[i] = encoder.getAngle() + angleOffsets[i]; // Read the angle from the encoder and store it in the array
  }
  return angles;
}

void setMotorPositions(std::array<double, 6> angles) {
  baseStepper.moveTo(angles[0] * baseGearRatio); // Move the base stepper to the specified angle
  shoulderStepper.moveTo(angles[1] * shoulderGearRatio); // Move the shoulder stepper to the specified angle
  primaryArmStepper.moveTo(angles[2] * primaryArmGearRatio); // Move the primary arm stepper to the specified angle
  wristRotationStepper.moveTo(angles[3] * wristGearRatio); // Move the wrist rotation stepper to the specified angle
  differential1Stepper.moveTo(angles[4] * differentialGearRatio); // Move the first differential stepper to the specified angle
  zWristRotationStepper.moveTo(angles[5] * zWristGearRatio); // Move the second differential stepper to the specified angle

}

void setup() {
  Wire1.begin(0);
  Mat4 endEffectorTransform = calculateDHEquations(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // Calculates the position at the initial starting position
  
  Serial.begin(115200);
}


void loop() {
  std::array<double, 6> angles = readAngles(); // Read the angles from the encoders
  Mat4 endEffectorTransform = calculateDHEquations(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
  
}

