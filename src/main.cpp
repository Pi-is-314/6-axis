#include <Arduino.h> // Required for PlatformIO/pioarduino projects
#include <AccelStepper.h> // Include the AccelStepper library for motor control
#include <math.h> // Include the math library for calculations
#include <Wire.h> // Include the Wire library for I2C communication
#include <ESP32Servo.h>
#include <DHEquations.h>
#include <array>

#define baseOffset 115.60000 // Base offset for the Z-axis (adjust if needed) (mm)
#define shoulderOffset 109.50000 // Offset of the shoulder segment (adjust if needed) (mm)
#define shoulderLength 175.00000 // Length of the shoulder segment (adjust if needed) (mm)
#define primaryArmLength 175.00000 // Length of the arm segment (adjust if needed) (mm)
#define secondaryArmLength 208.00000 // Length of the secondary arm segment (adjust if needed) (mm)
#define differentialOffset 94.88388 // Offset of the first differential segment (adjust if needed) (mm)

#define baseGearRatio 5.0 // Gear ratio for the base segment (adjust if needed)
#define shoulderGearRatio 10.0 // Gear ratio for the shoulder segment (adjust if needed)
#define primaryArmGearRatio 10.0 // Gear ratio for the primary arm segment (adjust if needed)
#define secondaryArmGearRatio 10.0 // Gear ratio for the secondary arm segment (adjust if needed)
#define differentialGearRatio 3.0 // Gear ratio for the differential segment (adjust if needed)

#define baseAngularOffset 0.0 // Angular offset for the base segment (radians) // Adjust if needed
#define shoulderAngularOffset 0.0 // Angular offset for the shoulder segment (radians) // Adjust if needed
#define primaryArmAngularOffset 0.0 // Angular offset for the primary arm segment (radians) // Adjust if needed
#define secondaryArmAngularOffset 0.0 // Angular offset for the secondary arm segment (radians) // Adjust if needed
#define differential1AngularOffset 0.0 // Angular offset for the first differential segment (radians) // Adjust if needed
#define differential2AngularOffset 0.0 // Angular offset for the second differential segment (radians) // Adjust if needed

double baseAngle = 0.0; // Current angle of the base segment (radians)
double shoulderAngle = 0.0; // Current angle of the shoulder segment (radians)
double primaryArmAngle = 0.0; // Current angle of the primary arm segment (radians)
double secondaryArmAngle = 0.0; // Current angle of the secondary arm segment (radians)
double differential1Angle = 0.0; // Current angle of the first differential segment (radians)
double differential2Angle = 0.0; // Current angle of the second differential segment (radians)
double differentialWristOrientation = differential1Angle + differential2Angle; // Current orientation of the differential segments (radians)
double differentialWristZOrientation = differential1Angle - differential2Angle; // Current orientation of the differential segments (radians)

//NEEDS TO BE CONSTANTLY UPDATED BASED ON THE CURRENT ANGLES OF THE SEGMENTS
Mat4 endEffectorTransform = calculateDHEquations(baseAngle, shoulderAngle, primaryArmAngle, secondaryArmAngle, differentialWristOrientation, differentialWristZOrientation);

AccelStepper baseStepper(AccelStepper::DRIVER, 2, 3); // Create a stepper object for the base segament (pins 2 and 3)
AccelStepper shoulderStepper(AccelStepper::DRIVER, 4, 5); // Create a stepper object for the shoulder segment (pins 4 and 5)
AccelStepper primaryArmStepper(AccelStepper::DRIVER, 6, 7); // Create a stepper object for the primary arm segment (pins 6 and 7)
AccelStepper secondaryArmStepper(AccelStepper::DRIVER, 8, 9); // Create a stepper object for the secondary arm segment (pins 8 and 9)
AccelStepper differential1Stepper(AccelStepper::DRIVER, 10, 11); // Create a stepper object for the first differential segment (pins 10 and 11)
AccelStepper differential2Stepper(AccelStepper::DRIVER, 12, 13); // Create a stepper object for the second differential
Servo differentialWristServo; // Create a servo object for the differential wrist segment



std::array<double, 3> getPositionVector() {
  std::array<double, 3> positionVector = {
    endEffectorTransform[0][3],
    endEffectorTransform[1][3],
    endEffectorTransform[2][3]
  };
  return positionVector;
}

std::array<double, 3> getRollPitchYaw() {
  std::array<double, 3> rollPitchYaw = {0.0, 0.0, 0.0}; //Roll, pitch, and yaw angles for the end effector
  // Convert the initial orientation vector to roll, pitch, and yaw angles
  rollPitchYaw[0] = atan2(endEffectorTransform[2][1], endEffectorTransform[2][2]); // Roll
  rollPitchYaw[1] = atan2(-endEffectorTransform[2][0], sqrt(pow(endEffectorTransform[0][0], 2) + pow(endEffectorTransform[1][0], 2))); // Pitch
  rollPitchYaw[2] = atan2(endEffectorTransform[1][0], endEffectorTransform[0][0]); // Yaw
  return rollPitchYaw;
} 


// Implement the FABRIK algorithm here to calculate the joint angles based on the target position and orientation
std::array<double, 6> fabrikCalculations(std::array<double, 3> currentPosition, std::array<double, 3> currentOrientation, std::array<double, 3> targetPosition, std::array<double, 3> targetOrientation) {
  double maxPositionalError = 0.1; // Maximum positional error allowed (mm)
  double maxRotationalError = 0.1; // Maximum rotational error allowed (radians)
  double positionalError = sqrt(pow(targetPosition[0] - currentPosition[0], 2) + pow(targetPosition[1] - currentPosition[1], 2) + pow(targetPosition[2] - currentPosition[2], 2)); // Calculate the positional error
  
  
  // This is a placeholder implementation and should be replaced with the actual FABRIK calculations
  std::array<double, 6> jointAngles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return jointAngles;
}
void setup() {
  Wire1.begin(0);
  Serial.begin(115200);
}

void loop() {
  Serial.println("Hello from ESP32-C6 via pioarduino!"); 
}