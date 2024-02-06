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
    }

    public static final class DriveConstants {

        public static final int kPigeonID = 23;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 2;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final int kFrontLeftDriveCanID = 1;
        public static final int kBackLeftDriveCanID = 3;
        public static final int kFrontRightDriveCanID = 7;
        public static final int kBackRightDriveCanID = 5;

        public static final int kFrontLeftTurningCanID = 2;
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
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;
    }

}

