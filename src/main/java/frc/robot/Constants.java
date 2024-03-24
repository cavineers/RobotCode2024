package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

        public static final int LeftClimberCanID = 19; //19
        public static final int RightClimberCanID = 16; //16

        public static final int PDHCanID = 17; 

        public static final int ShooterCanID = 13; 
        public static final int UpperIntakeCanID = 21; 
        public static final int LowerIntakeCanID = 20; 

        public static final int kPigeonID = 23;
    }

        

    public static final class DIO {
        public static final int ArmBoreEncoder = 0;

        public static final int GantryLowerLimitSwitch = 1;
        public static final int GantryHigherLimitSwitch = 2;

        public static final int LeftClimberLimitSwitch = 6; 
        public static final int RightClimberLimitSwitch = 5;

        public static final int NoteSensorLeft = 4;
        public static final int NoteSensorRight = 3;
    }

    public static final class DriveConstants {

        public static final double kPhysicalMaxSpeedMetersPerSecond = 2;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

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
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kFrontLeftAbsoluteEncoderOffset = -.0083+.5; //.281 185.97 offset //-5.97 +80.63
        public static final double kBackLeftAbsoluteEncoderOffset = -0.0920+.5; //-.472 178.41 //-178.41 -224.03
        public static final double kFrontRightAbsoluteEncoderOffset = -0.9697; //170.94 //-170.94
        public static final double kBackRightAbsoluteEncoderOffset = -.723+.5; //58.79 //-58.79
        
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
        public static final double kDeadband = 0.046;
        public static final double kDriverJoystickTriggerDeadzone = 0.1;
    }

    public static final class ArmPivot {
    
        public static final double PivotMotorSpeedForwards = 0.3;
        public static final double PivotMotorSpeedBackwards = -0.1;
    
        public static final double PivotMotorLowerRotationLimit = .353 - 0.007; //Required For Auto Shoot
        public static final double PivotMotorUpperRotationLimit = 0.736 - 0.007; //Required For Auto Shoot
    
        public static final double ArmPivotEcoderDeadzone = 0.4;
    
        public static final double DistancePerRotation = 1;

        public static final double PivotSetpointTolerance = 0.02;
    
        public static final double GroundPickupRotations = 0.353 - 0.007;
        public static final double RestRotations = 0.342 - 0.007;
        public static final double ShootCloseRotations = 0.415 - 0.007; //0.401
        public static final double ShootGroundRotations = 0.42;
        public static final double SourceRotations = 0.6138 - 0.007;
        public static final double AmpRotations = 0.644 - 0.007;

        public static final double ProportionalGain = 3.4; // strength of a correction
        public static final double IntegralTerm = 0; // additive strength over time
        public static final double DerivitiveTerm = 0.0;

        public static double MotorSetPoint = 0;

        //Measurements
        public static final double armPivotRadiusMeters = .38; //Required For Auto Shoot
        public static final double armPivotJointAngleDegrees = 108; //Required For Auto Shoot
        public static final double armPivotMinAngleDegrees = 10; //Required For Auto Shoot
        public static final double armPivotMaxAngleDegrees = 153; ////Required For Auto Shoot

        public static final double dAngle = armPivotMaxAngleDegrees - armPivotMinAngleDegrees;
        public static final double dRotations = PivotMotorUpperRotationLimit - PivotMotorLowerRotationLimit;

        public static final double PivotRestMinRotations = 0.34 - 0.007;
        public static final double PivotNormalMinRotations = 0.41 - 0.007;
        public static final double PivotGroundMinRotations = 0.353 - 0.007;
    
    }
    public static final class ArmBase {

        public static final double SpeedForwards = 0.2;
        public static final double SpeedBackwards = -0.15;

        public static final double MaxRotations = 151;
        public static final double MinRotations = 0;

        public static final double ArmBaseEncoderDeadzone = 0.4;

        public static final double BaseMotorEaseFactor = 0.1;

        public static final double BaseSetpointTolerance = 1;

        public static final double GroundPickupRotations = 0;
        public static final double RestRotations = 151;
        public static final double ShootRotations = 151;
        public static final double ShootGroundRotations = 0;
        public static final double SourceRotations = 0;
        public static final double AmpRotations = 151;

        public static final double ProportionalGain = .1; // strength of a correction
        public static final double IntegralTerm = 0.00; // additive strength over time
        public static final double DerivitiveTerm = 0.0;

        //Measurements
        public static final double minGantryHeightMeters = 0.025; //TBD
        public static final double maxGantryHeightMeters = 0.533; //TBD
        
        public static final double dHeight = maxGantryHeightMeters - minGantryHeightMeters;
        public static final double dRotations = MaxRotations - MinRotations;

        public static final double PivotRegionRestMin = 149;
        public static final double PivotRegionGroundMax = 1;
    }

    public static final class Climber {

        public static final double ClimberExtensionSpeed = 20; // TBD
        public static final double ClimberExtensionSpeedRev = -20; // TBD
        public static final double LowerClimberMaxRotations = 0; // TBD
        public static final double UpperClimberMaxRotations = 150; // TBD

        public static final double ProportionalGain = .05;
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
        public static final double UpperIntakeForwardSpeed = .77; // TBD
        public static final double LowerIntakeForwardSpeed = .77; // TBD
        public static final double UpperIntakeReverseSpeed = -.77; // TBD
        public static final double LowerIntakeReverseSpeed = -.77; // TBD
        public static final double UpperIntakeRetractSpeed = -.05; // TBD
        public static final double LowerIntakeRetractSpeed = -.05; // TBD
    }

    public static final class VisionConstants {
        public static final Transform3d LeftCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-11.88), Units.inchesToMeters(-6.88), Units.inchesToMeters(31.09)),
            new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(-45)));
        
        public static final Transform3d RightCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-11.88), Units.inchesToMeters(-6.88), Units.inchesToMeters(31.09)),
            new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(-45)));

         public static final Transform3d FrontCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-11.88), Units.inchesToMeters(-6.88), Units.inchesToMeters(31.09)),
            new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(-45)));
        
        public static final double blueSpeakerX = Units.inchesToMeters(-1.5);
        public static final double redSpeakerX = Units.inchesToMeters(652.73);
        public static final double blueSpeakerY = Units.inchesToMeters(218.42);
        public static final double redSpeakerY = Units.inchesToMeters(218.42);

        // Auto shoot maximum and minimums in meters
        public static double shootDistanceMinimum = Units.metersToInches(0.5);
        public static double shootDistanceMaximum = Units.metersToInches(4.57);
    }


}
