// Constants
const float leadGantryX = 0.044429f;        // Lead of the parallel worm screws in m per revolution
const float leadGantryZ = 0.02f;
const float leadGantryY = 0.06f;

const float GantryXLinearPosPerRad = leadGantryX / (2.0f * _PI);  // Linear movement per radian for Gx
const float GantryZLinearPosPerRad = leadGantryZ / (2.0f * _PI);  // Linear movement per radian for Gx
const float GantryYLinearPosPerRad = leadGantryY / (2.0f * _PI);  // Linear movement per radian for Gx

// Input
float GantryYLinearPos // Linear position in m 
float GantryXLinearPos // = 0.5f * GantryXLinearPosPerRad * (GantryX1AngleRad + GantryX2AngleRad);
float GantryXRotAngleRad // = 2/3 * (GantryX2AngleRad - GantryX1AngleRad);
float GantryZLinearPos

// Output
float GantryYAngleRad = GantryYLinearPos / GantryYLinearPosPerRad // motorID = AA, feedbackID = BA
float GantryX1AngleRad = GantryXLinearPos / GantryXLinearPosPerRad - 0.75f * GantryXRotAngleRad // motorID = AC, feedbackID = BC
float GantryX2AngleRad = GantryXLinearPos / GantryXLinearPosPerRad + 0.75f * GantryXRotAngleRad // motorID = AB, feedbackID = BB
float GantryZAngleRad = GantryZLinearPos / GantryZLinearPosPerRad // motorID = AD, feedbackID = BD

// Scaling constant for force feedback intensity (adjust as needed)
float K = /* Your scaling constant value */;

// Encoder readings (in radians)
// Replace the placeholders with your actual encoder reading functions or variables
float Gx1Angle = /* Encoder reading for motor Gx1 in radians */;
float Gx2Angle = /* Encoder reading for motor Gx2 in radians */;
float mArmZAngle = /* Encoder reading for mArmZ (MArm linear position) in radians */;
float mArmZrotAngle = /* Encoder reading for mArmZrot (MArm rotational position) in radians */;

// Calculations

// First System (Perpendicular Worm Screw)
float GantryxLinearPos_mm = GantryxLinearPosPerRad * (Gantryx1AngleRad + Gantryx2AngleRad);          // Linear position in mm
float GantryxRotAngleRad = 3/2 * (Gantryx1AngleRad - Gantryx2AngleRad);                              // Rotational position in radians

// Second System (MArm Gantry)
float MArmLinearPos_mm = MArmLinearPerRad * mArmZAngle;                 // Linear position in mm
float MArmRotAngleRad = mArmZrotAngle;                                  // Rotational position in radians

// Error Calculations
float linearPosError_mm = MArmLinearPos_mm - GxLinearPos_mm;            // Linear position error in mm
float rotAngleErrorRad = MArmRotAngleRad - GxRotAngleRad;               // Rotational position error in radians

// Control Equations for Motors

// Motor Torques for Gx1 and Gx2
float torqueGx1 = -K * (linearPosError_mm + rotAngleErrorRad);          // Torque for motor Gx1
float torqueGx2 = -K * (linearPosError_mm - rotAngleErrorRad);          // Torque for motor Gx2

// MArm Gantry Motor Torques
float torque_mArmZ = K * linearPosError_mm;                             // Torque for MArm's linear motor (mArmZ)
float torque_mArmZrot = K * rotAngleErrorRad;                           // Torque for MArm's rotational motor (mArmZrot)

// Apply Torques to Motors
// Replace the placeholders with your motor control functions or commands
torqueToMotorGx1(torqueGx1);               // Function to send and apply torque to motor Gx1
torqueToMotorGx2(torqueGx2);               // Function to send and apply torque to motor Gx2
torqueToMotor_mArmZ(torque_mArmZ);         // Function to send and apply torque to MArm's linear motor (mArmZ)
torqueToMotor_mArmZrot(torque_mArmZrot);   // Function to apply torque to MArm's rotational motor (mArmZrot)
