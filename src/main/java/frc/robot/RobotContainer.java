package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.OIConstants;

import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveHoming;
import frc.robot.Robot;


public class RobotContainer {

    private final SwerveDriveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final Joystick driverJoystick;
    private final JoystickButton button;
    public JoystickButton l_bump;

    public SwerveHoming swerveHomingCommand;

    public RobotContainer() {

        visionSubsystem = new VisionSubsystem();
        swerveSubsystem = new SwerveDriveSubsystem(visionSubsystem);
        
        swerveHomingCommand = new SwerveHoming(swerveSubsystem);

        driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
        button = new JoystickButton(driverJoystick, 4);
        l_bump = new JoystickButton(driverJoystick, 5);
        

        swerveSubsystem.setDefaultCommand(new SwerveCommand(
            swerveSubsystem,
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();
    };

    private void configureButtonBindings() {

    }   

    public SwerveDriveSubsystem getSwerveSubsystem() {
        return this.swerveSubsystem;
    }
    public VisionSubsystem getVisionSubsystem() {
        return this.visionSubsystem;
    }
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("TestAutoNow");
    }

}
