package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveHoming;
import frc.robot.commands.Arm.ManualRaise;
import frc.robot.commands.Arm.ArmPreset;
import frc.robot.commands.Arm.ManualLower;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;


public class RobotContainer {

    private final SwerveDriveSubsystem swerveSubsystem;

    public Command groundPreset;

    private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverJoystickPort);

    public SwerveHoming swerveHomingCommand;

    public RobotContainer() {

        groundPreset = new ArmPreset(Constants.ArmBase.GroundPositionRotations, Constants.ArmPivot.PivotMotorGroundRotations);

        swerveSubsystem = new SwerveDriveSubsystem();

        swerveHomingCommand = new SwerveHoming(swerveSubsystem);
        

        // swerveSubsystem.setDefaultCommand(new SwerveCommand(
        //     swerveSubsystem,
        //     () -> -m_driverController.getLeftY(),
        //     () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
        //     () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
        //     () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();
    };

    private void configureButtonBindings() {
        m_driverController.a().whileTrue(new ManualLower(Robot.armBase));
        m_driverController.y().whileTrue(new ManualRaise(Robot.armBase));
    }   

    public SwerveDriveSubsystem getSwerveSubsystem() {
        return this.swerveSubsystem;
    }

}
