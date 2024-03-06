package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

    public static final class FieldConstants {
        public static final double heightToSpeakerPointMeters = 2.1336;
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / (150.0 / 7);
        public static final double kTurningDegreesToRad = Math.PI / 180;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class CanIDs{

        public static final int kFrontLeftDriveCanID = 2;
        public static final int kBackLeftDriveCanID = 8;
        public static final int kFrontRightDriveCanID = 4;
        public static final int kBackRightDriveCanID = 6;

        public static final int kFrontLeftTurningCanID = 1;
        public static final int kBackLeftTurningCanID = 7;
        public static final int kFrontRightTurningCanID = 3;
        public static final int kBackRightTurningCanID = 5;

        public static final int kFrontLeftAbsoluteEncoderPort = 10;
        public static final int kBackLeftAbsoluteEncoderPort = 11;
        public static final int kFrontRightAbsoluteEncoderPort = 9;
        public static final int kBackRightAbsoluteEncoderPort = 12;

        public static final int PivotCanID = 18;
        public static final int GantryCANID = 15;

        public static final int LeftClimberCanID = 19; 
        public static final int RightClimberCanID = 16; 

        public static final int PDHCanID = 17; 

        public static final int ShooterCanID = 13; 
        public static final int UpperIntakeCanID = 21; 
        public static final int LowerIntakeCanID = 20; 

        public static final int kPigeonID = 23;
    }

        

    public static final class DIO {
        
        public static final int ArmBoreEncoder = 8;

        public static final int LeftGantryLowerLimitSwitch = 1;
        public static final int LeftGantryHigherLimitSwitch = 2;
        public static final int RightGantryLowerLimitSwitch = 3;
        public static final int RightGantryHigherLimitSwitch = 4;

        public static final int LeftClimberBottomLimitSwitch = 10; 
        public static final int RightClimberBottomLimitSwitch = 6;

        public static final int NoteSensor = 5;
        // public static final int IntakeSwitch = 8;

    }

    public static final class DriveConstants {

        public static final double kPhysicalMaxSpeedMetersPerSecond = 2;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        // public static final int kFrontLeftAbsoluteEncoderPort = 12;
        // public static final int kBackLeftAbsoluteEncoderPort = 9;
        // public static final int kFrontRightAbsoluteEncoderPort = 11;
        // public static final int kBackRightAbsoluteEncoderPort = 10;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMinSpeedMetersPerSecond = 5 / 3;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kFrontLeftAbsoluteEncoderOffset = -.0083+.5; //.281 185.97 offset //-5.97 +80.63
        public static final double kBackLeftAbsoluteEncoderOffset = -0.0920+.5; //-.472 178.41 //-178.41 -224.03
        public static final double kFrontRightAbsoluteEncoderOffset = -0.9697; //170.94 //-170.94
        public static final double kBackRightAbsoluteEncoderOffset = -.723+.5;
         //58.79 //-58.79
        
        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(25); 
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(25);
        
     
        public static final SwerveDriveKinematics SwerveKinematics = new SwerveDriveKinematics(
            new Translation2d(DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0),
            new Translation2d(DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0),
            new Translation2d(-DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0),
            new Translation2d(-DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0));
    }

    public static final class OIConstants {
        public static final int kDriverJoystickPort = 0;
        public static final int kSecondDriverJoystickPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverLeftTriggerAxis = 2;
        public static final int kDriverRightTriggerAxis = 3;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kTriggerDeadzone = 0.1;
        public static final double kDeadband = 0.03;
        public static final double kDriverJoystickTriggerDeadzone = 0.1;

    }

    public static final class ArmPivot {
    
        public static final double PivotMotorSpeedForwards = 0.3;
        public static final double PivotMotorSpeedBackwards = -0.1;
    
        public static final double PivotMotorLowerRotationLimit = .353;
        public static final double PivotMotorUpperRotationLimit = 0.736;
    
        public static final double ArmPivotEcoderDeadzone = 0.4;
    
        public static final double DistancePerRotation = 1;
    
        public static final double GroundPickupRotations = 5;
        public static final double RestRotations = 10;
        public static final double ShootRotations = 15;
        public static final double SourceRotations = 20;
        public static final double AmpRotations = 25;

        public static final double ProportionalGain = 1.75; // strength of a correction
        public static final double IntegralTerm = 0.00; // additive strength over time
        public static final double DerivitiveTerm = 0.0;

        public static double MotorSetPoint = 0;

        //Measurements
        public static final double armPivotRadiusMeters = .38;
        public static final double armPivotJointAngleDegrees = 108; //TBD
        public static final double armPivotMinAngleDegrees = 11; //TBD
        public static final double armPivotMaxAngleDegrees = 155; //TBD

        public static final double dAngle = armPivotMaxAngleDegrees - armPivotMinAngleDegrees;
        public static final double dRotations = PivotMotorUpperRotationLimit - PivotMotorLowerRotationLimit;

        // ROTATIONS
        public static final double[] ArmPivotRotationGround = {0.35,0.4}; // [min, max] region 1
        public static final double[] ArmPivotRotationSwerve = {0.4,0.45}; // [min, max] region 2
        public static final double[] ArmPivotRotationMidGantry = {0.45,0.5}; // [min, max] region 3
        public static final double[] ArmPivotRotationUpperGantry = {0.5,0.55}; // [min, max] region 4
    
    }
    public static final class ArmBase {

        public static final double SpeedForwards = 0.2;
        public static final double SpeedBackwards = -0.15;

        public static final double MaxRotations = 150;
        public static final double MinRotations = 0;

        public static final double ArmBaseEncoderDeadzone = 0.4;

        public static final double BaseMotorEaseFactor = 0.1;

        public static final double GroundPickupRotations = 10;
        public static final double RestRotations = 13;
        public static final double ShootRotations = 18;
        public static final double SourceRotations = 21;
        public static final double AmpRotations = 25;

        public static final double ProportionalGain = .03; // strength of a correction
        public static final double IntegralTerm = 0.00; // additive strength over time
        public static final double DerivitiveTerm = 0.0;

        //Measurements
        public static final double minGantryHeightMeters = 0.025; //TBD
        public static final double maxGantryHeightMeters = 0.533; //TBD
        
        public static final double dHeight = maxGantryHeightMeters - minGantryHeightMeters;
        public static final double dRotations = MaxRotations - MinRotations;

        // REGIONS
        public static final double[] ArmPivotRegionGround = {0,25}; // [min, max] region 1
        public static final double[] ArmPivotRegionSwerve = {25,50}; // [min, max] region 2
        public static final double[] ArmPivotRegionMidGantry = {75,100}; // [min, max] region 3
        public static final double[] ArmPivotRegionUpperGantry = {100,150}; // [min, max] region 4
    }

    public static final class Climber {

        public static final double ClimberExtensionSpeed = 20; // TBD
        public static final double ClimberExtensionSpeedRev = -20; // TBD
        public static final double LowerClimberMaxRotations = 0; // TBD
        public static final double UpperClimberMaxRotations = 100; // TBD

        public static final double ProportionalGain = .03;
        public static final double IntegralTerm = 0.00;
        public static final double DerivitiveTerm = 0.0;

    }

    public static final class Shooter {

        public static final double ShooterForwardSpeed = .85; // TBD
        public static final double AmpForwardSpeed = .35;
        public static final double ShooterReverseSpeed = -.5; // TBD

        public static double kP = 0.01; // Proportional
        public static double kI = 0.3; // Integral
        public static double kD = 0.01; // Derivative
        public static double kF = 0.000204; // Feed Forward

        public static final double shootingVertexHeightMeters = 2.0574;

        //Measurements
        public static final double shooterAngleFromArmPivotDegrees = 180 - ArmPivot.armPivotJointAngleDegrees;
    }

    public static final class Intake {
        public static final double UpperIntakeForwardSpeed = .7; // TBD
        public static final double LowerIntakeForwardSpeed = .7; // TBD
        public static final double UpperIntakeReverseSpeed = -.7; // TBD
        public static final double LowerIntakeReverseSpeed = -.7; // TBD
        public static final double UpperIntakeRetractSpeed = -.05; // TBD
        public static final double LowerIntakeRetractSpeed = -.05; // TBD
    }

}
