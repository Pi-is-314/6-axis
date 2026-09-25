#include <Arduino.h> // Required for PlatformIO/pioarduino projects
#include <AccelStepper.h> // Include the AccelStepper library for motor control
#include <math.h> // Include the math library for calculations
#include <Wire.h> // Include the Wire library for I2C communication
#include <servo.h> // Include the Servo library for servo control


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

std::array<double, 3> baseVector = {cos(baseAngle) * shoulderOffset, sin(baseAngle) * shoulderOffset, baseOffset}; // Current position of the end effector (X, Y, Z) in mm.
std::array<double, 3> shoulderVector = {cos(baseAngle) * cos(shoulderAngle) * shoulderLength, sin(baseAngle) * cos(shoulderAngle) * shoulderLength, sin(shoulderAngle) * shoulderLength}; // Current position of the end effector (X, Y, Z) in mm.
std::array<double, 3> primaryArmVector = {cos(baseAngle) * cos(shoulderAngle + primaryArmAngle) * primaryArmLength, sin(baseAngle) * cos(shoulderAngle + primaryArmAngle) * primaryArmLength, sin(shoulderAngle + primaryArmAngle) * primaryArmLength}; // Current position of the end effector (X, Y, Z) in mm.
std::array<double, 3> secondaryArmVector = {cos(baseAngle) * cos(shoulderAngle + primaryArmAngle + secondaryArmAngle) * secondaryArmLength, sin(baseAngle) * cos(shoulderAngle + primaryArmAngle + secondaryArmAngle) * secondaryArmLength, sin(shoulderAngle + primaryArmAngle + secondaryArmAngle) * secondaryArmLength}; // Current position of the end effector (X, Y, Z) in mm.
std::array<double, 3> differentialVector = {cos(baseAngle) * cos(shoulderAngle + primaryArmAngle + secondaryArm_ANGLE + differentialWristOrientation) * differentialOffset, sin(base_angle) * cos(shoulder_angle + primary_arm_angle + secondary_arm_angle + differential_wrist_orientation) * differential_offset, sin(shoulder_angle + primary_arm_angle + secondary_arm_angle + differential_wrist_orientation) * differential_offset}; // Current position of the end effector (X, Y, Z) in mm.

stepper baseStepper(AccelStepper::DRIVER, 2, 3); // Create a stepper object for the base segment (pins 2 and 3)
stepper shoulderStepper(AccelStepper::DRIVER, 4, 5); // Create a stepper object for the shoulder segment (pins 4 and 5)
stepper primaryArmStepper(AccelStepper::DRIVER, 6, 7); // Create a stepper object for the primary arm segment (pins 6 and 7)
stepper secondaryArmStepper(AccelStepper::DRIVER, 8, 9); // Create a stepper object for the secondary arm segment (pins 8 and 9)
stepper differential1Stepper(AccelStepper::DRIVER, 10, 11); // Create a stepper object for the first differential segment (pins 10 and 11)
stepper differential2Stepper(AccelStepper::DRIVER, 12, 13); // Create a stepper object for the second differential  
servo differentialWristServo; // Create a servo object for the differential wrist segment



std::array<double, 3> getPositionVector() {
  std::array<double, 3> positionVector = {
    baseVector[0] + shoulderVector[0] + primaryArmVector[0] + secondaryArmVector[0] + differentialVector[0]
  , baseVector[1] + shoulderVector[1] + primaryArmVector[1] + secondaryArmVector[1] + differentialVector[1]
  , baseVector[2] + shoulderVector[2] + primaryArmVector[2] + secondaryArmVector[2] + differentialVector[2]};
  return positionVector;
}
std::array<double, 3> getOrientationVector() {
  std::array<double, 3> orientationVector = {shoulderAngle + primaryArmAngle + secondaryArmAngle, differentialWristOrientation, differentialWristZOrientation + baseAngle};
  return orientationVector;
} 

std::array<double, 3> orientationVectorConversion(){
  std::array<double, 3> unitVector = {}
} 
// Implement the FABRIK algorithm here to calculate the joint angles based on the target position and orientation
std::array<double, 6> fabrikCalculations(std::array<double, 3> currentPosition, std::array<double, 3> currentOrientation, std::array<double, 3> targetPosition, std::array<double, 3> targetOrientation) {
  maxPositionalError = 0.1; // Maximum positional error allowed (mm)
  maxRotationalError = 0.1; // Maximum rotational error allowed (radians)
  positionalError = sqrt(pow(targetPosition[0] - currentPosition[0], 2) + pow(targetPosition[1] - currentPosition[1], 2) + pow(targetPosition[2] - currentPosition[2], 2)); // Calculate the positional error
  rotationalError = sqrt(pow(targetOrientation[0] - currentOrientation[0], 2) + pow(targetOrientation[1] - currentOrientation[1], 2) +
pow(targetOrientation[2] - currentOrientation[2], 2)); // Calculate the rotational error
` while(positionalError > maxPositionalError || rotationalError > maxRotationalError) {
    // Perform FABRIK iterations to adjust joint angles
    // Update currentPosition and currentOrientation based on the new joint angles
    // Recalculate positionalError and rotationalError
    
  }
  
  // This is a placeholder implementation and should be replaced with the actual FABRIK calculations
  std::array<double, 6> jointAngles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return jointAngles;
}
void setup() {
  wire.begin(0);
  Serial.begin(115200);
}

void loop() {
  Serial.println("Hello from ESP32-C6 via pioarduino!");
}