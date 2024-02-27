package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

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

        public static final int PivotCanID = 15;
        public static final int GantryCANID = 100000;

        public static final int LeftClimberCanID = 21; 
        public static final int RightClimberCanID = 14; 

        public static final int ShooterCanID = 22; 
        public static final int UpperIntakeCanID = 16; 
        public static final int LowerIntakeCanID = 77; 

        public static final int kPigeonID = 23;
    }

        

    public static final class DIO {
        
        public static final int ArmBoreEncoder = 0;
        public static final int GantryLowerLimitSwitch = 1;
        public static final int GantryHigherLimitSwitch = 2;

        public static final int LeftClimberTopLimitSwitch = 3; 
        public static final int RightClimberTopLimitSwitch = 4; 
        public static final int LeftClimberBottomLimitSwitch = 5; 
        public static final int RightClimberBottomLimitSwitch = 6;

        public static final int NoteSensor = 7;
        public static final int IntakeSwitch = 8;

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
        public static final double kDeadband = 0.1;
        public static final double kDriverJoystickTriggerDeadzone = 0.1;

    }

    public static final class ArmPivot {
    
        public static final double PivotMotorSpeedForwards = 0.3;
        public static final double PivotMotorSpeedBackwards = -0.1;
    
        public static final double PivotMotorLowerRotationLimit = 0;
        public static final double PivotMotorUpperRotationLimit = 40;
    
        public static final double ArmPivotEcoderDeadzone = 0.4;
    
        public static final double DistancePerRotation = 1;
    
        public static final double GroundPickupRotations = 10;
        public static final double RestRotations = 15;
        public static final double ShootRotations = 20;
        public static final double SourceRotations = 30;
        public static final double AmpRotations = 40;

        public static final double ProportionalGain = .06; // strength of a correction
        public static final double IntegralTerm = 0.00; // additive strength over time
        public static final double DerivitiveTerm = 0.0;

        public static double MotorSetPoint = 0;
    
    }

    public static final class ArmBase {

        public static final double SpeedForwards = 0.2;
        public static final double SpeedBackwards = -0.15;

        public static final double MaxRotations = 95;
        public static final double MinRotations = 0;

        public static final double ArmBaseEncoderDeadzone = 0.4;

        public static final double BaseMotorEaseFactor = 0.1;

        public static final double GroundPickupRotations = 15;
        public static final double RestRotations = 20;
        public static final double ShootRotations = 25;
        public static final double SourceRotations = 35;
        public static final double AmpRotations = 45;

        public static final double ProportionalGain = .03; // strength of a correction
        public static final double IntegralTerm = 0.00; // additive strength over time
        public static final double DerivitiveTerm = 0.0;

    }

    public static final class Climber {

        public static final double ClimberExtensionSpeed = 20; // TBD
        public static final double ClimberExtensionSpeedRev = -20; // TBD
        public static final double LowerClimberMaxRotations = 4; // TBD
    }

    public static final class ShooterIntake {

        public static final double ShooterForwardSpeed = 1; // TBD
        public static final double ShooterReverseSpeed = -.5; // TBD
        public static final double UpperIntakeForwardSpeed = 1; // TBD
        public static final double LowerIntakeForwardSpeed = 1; // TBD
        public static final double UpperIntakeReverseSpeed = -1; // TBD
        public static final double LowerIntakeReverseSpeed = -1; // TBD
        public static final double UpperIntakeRetractSpeed = -.05; // TBD
        public static final double LowerIntakeRetractSpeed = -.05; // TBD
    }

}
