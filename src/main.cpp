#include <Arduino.h> // Required for PlatformIO/pioarduino projects
#include <AccelStepper.h> // Include the AccelStepper library for motor control
#include <math.h> // Include the math library for calculations
#include <Wire.h> // Include the Wire library for I2C communication

#define baseOffset 115.60000 // Base offset for the Z-axis (adjust if needed) (mm)
#define shoulderOffset 109.50000 // Offset of the shoulder segment (adjust if needed) (mm)
#define shoulderLength 175.00000 // Length of the shoulder segment (adjust if needed) (mm)
#define primaryArmLength 175.00000 // Length of the arm segment (adjust if needed) (mm)
#define secondaryArmLength 208.00000 // Length of the secondary arm segment (adjust if needed) (mm)
#define differentialOffset 0.00000 // Offset of the first differential segment (adjust if needed) (mm)

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

double baseVector[3] = {cos(baseAngle) * shoulderOffset, sin(baseAngle) * shoulderOffset, baseOffset}; // Current position of the end effector (X, Y, Z) in mm.
double shoulderVector[3] = {cos(baseAngle) * cos(shoulderAngle) * shoulderLength, sin(baseAngle) * cos(shoulderAngle) * shoulderLength, sin(shoulderAngle) * shoulderLength}; // Current position of the end effector (X, Y, Z) in mm.
double primaryArmVector[3] = {cos(baseAngle) * cos(shoulderAngle + primaryArmAngle) * primaryArmLength, sin(baseAngle) * cos(shoulderAngle + primaryArmAngle) * primaryArmLength, sin(shoulderAngle + primaryArmAngle) * primaryArmLength}; // Current position of the end effector (X, Y, Z) in mm.
double secondaryArmVector[3] = {cos(baseAngle) * cos(shoulderAngle + primaryArmAngle + secondaryArmAngle) * secondaryArmLength, sin(baseAngle) * cos(shoulderAngle + primaryArmAngle + secondaryArmAngle) * secondaryArmLength, sin(shoulderAngle + primaryArmAngle + secondaryArmAngle) * secondaryArmLength}; // Current position of the end effector (X, Y, Z) in mm.
double differentialVector[3] = {cos(baseAngle) * cos(shoulderAngle + primaryArmAngle + secondaryArmAngle + differentialWristOrientation) * differentialOffset, sin(baseAngle) * cos(shoulderAngle + primaryArmAngle + secondaryArmAngle + differentialWristOrientation) * differentialOffset, sin(shoulderAngle + primaryArmAngle + secondaryArmAngle + differentialWristOrientation) * differentialOffset}; // Current position of the end effector (X, Y, Z) in mm.

double positionVector[3] = {baseVector[0] + shoulderVector[0] + primaryArmVector[0] + secondaryArmVector[0], baseVector[1] + shoulderVector[1] + primaryArmVector[1] + secondaryArmVector[1], baseVector[2] + shoulderVector[2] + primaryArmVector[2] + secondaryArmVector[2]};
double oreientationVector[3] = {0,0,0};














void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println("Hello from ESP32-C6 via pioarduino!");
}