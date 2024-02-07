package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1/ 6.75;
        public static final double kTurningMotorGearRatio = 1/ (150.0/7);
        public static final double kTurningDegreesToRad = Math.PI/180;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60; 
        public static final double kPTurning = 0.5;
    }
    
    public static final class DIO {

        public static int IntakeSwitch = 7;
        public static int LeftClimberTopLimitSwitch = 1; //TBD
        public static int RightClimberTopLimitSwitch = 2; //TBD
        public static int LeftClimberBottomLimitSwitch = 3; //TBD
        public static int RightClimberBottomLimitSwitch = 4; //TBD
        public static int noteSensor = 0;
    }

    public static final class DriveConstants {

        public static final double kPhysicalMaxSpeedMetersPerSecond = 2;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        // public static final int kFrontLeftDriveCanID = 1;
        // public static final int kBackLeftDriveCanID = 3;
        // public static final int kFrontRightDriveCanID = 7;
        // public static final int kBackRightDriveCanID = 5;

        // public static final int kFrontLeftTurningCanID = 2;
        public static final int kBackLeftTurningCanID = 4;
        public static final int kFrontRightTurningCanID = 8;
        public static final int kBackRightTurningCanID = 6;

        public static final int kFrontLeftAbsoluteEncoderPort = 12;
        public static final int kBackLeftAbsoluteEncoderPort = 9;
        public static final int kFrontRightAbsoluteEncoderPort = 11;
        public static final int kBackRightAbsoluteEncoderPort = 10;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final boolean kFrontLeftAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftAbsoluteEncoderReversed = true; //TBD
        public static final boolean kFrontRightAbsoluteEncoderReversed = false; //TBD
        public static final boolean kBackRightAbsoluteEncoderReversed = false; //TBD

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMinSpeedMetersPerSecond = 5/3;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kFrontLeftAbsoluteEncoderOffset = 176.49; //185.97 offset //-5.97 +80.63
        public static final double kBackLeftAbsoluteEncoderOffset = 180.44; //178.41 //-178.41 -224.03
        public static final double kFrontRightAbsoluteEncoderOffset = -171.56; //170.94 //-170.94
        public static final double kBackRightAbsoluteEncoderOffset = -57.65; //58.79 //-58.79
        
        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(23.75); 
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(24.75);
       
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
    }

    public static final class OIConstants {
        public static final int kDriverJoystickPort = 0;
        
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

        public static final int PivotMotor = 3;
        public static final int ArmPivotEncoder = 9;

        public static final double PivotMotorSpeedForwards = 1;
        public static final double PivotMotorSpeedBackwards = -1;

        public static final double PivotMotorLowerRotationLimit = -10;
        public static final double PivotMotorUpperRotationLimit = 10;

        public static final double ArmPivotEcoderDeadzone = 0.1;
        
        public static final double DistancePerRotation = 1;

        public static final double GroundPickupRotations = 10;

    }

    public static final class ArmBase {

        public static final int BaseMotorCANID = 1;
        public static final int lowerLimitSwitchPort = 1;
        public static final int higherLimitSwitchPort = 0;

        public static final double SpeedForwards = 0.1;
        public static final double SpeedBackwards = -0.1;

        public static final double MaxRotations = 10;
        public static final double MinRotations = 0;

        public static final double GroundPositionRotations = 10;
        public static final double SpeakerPositionRotations = 10;

        public static final double ArmBaseEcoderDeadzone = 0.1;

        public static final double GroundPickupRotations = 15;

    }

    public static final class Climber {
        public static final int LeftClimberMotor = 1; //TBD
        public static final int RightClimberMotor = 2; //TBD
        public static final double ClimberExtensionSpeed = 20; //TBD
        public static final double ClimberExtensionSpeedRev = -20; //TBD
        public static final double LowerClimberMaxRotations= 4; //TBD
    }

    public static final class ShooterIntake {

        public static final int ShooterCanID = 2; //TBD
        public static final int IntakeCanID = 3; //TBD
        public static final int Intake2ndCanID = 4; //TBD

        public static final double ShooterForwardSpeed = 1; //TBD
        public static final double ShooterReverseSpeed = -.5; //TBD
        public static final double IntakeForwardSpeed = .3; //TBD
        public static final double IntakeReverseSpeed = -.3; //TBD
        public static final double IntakeRetractSpeed = -.05; //TBD     
    }

}

