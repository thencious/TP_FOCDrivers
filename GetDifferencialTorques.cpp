// Constants
const float leadWormSc = 44.429f;        // Lead of the parallel worm screws in mm per revolution
const float leadMArm = 19.1f * _PI;      // Lead of the MArm's linear guide rail in mm per revolution

const float GxLinearPerRad = leadWormSc / (2.0f * _PI);  // Linear movement per radian for Gx
const float MArmLinearPerRad = leadMArm / (2.0f * _PI);  // Linear movement per radian for the MArm gantry

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
float GxLinearPos_mm = GxLinearPerRad * (Gx1Angle + Gx2Angle);          // Linear position in mm
float GxRotAngleRad = Gx1Angle - Gx2Angle;                              // Rotational position in radians

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
