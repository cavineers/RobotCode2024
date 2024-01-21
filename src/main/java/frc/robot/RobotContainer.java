package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SampleForward;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveHoming;
import frc.robot.Robot;


public class RobotContainer {

    public Command sampleForward;
    public Command sampleBackward;

    public Joystick joy = new Joystick(0);
    public JoystickButton y_button = new JoystickButton(joy, 1);
    public JoystickButton a_button = new JoystickButton(joy, 2);

    private final SwerveDriveSubsystem swerveSubsystem;


    private final Joystick driverJoystick;
    private final JoystickButton button;
    public JoystickButton l_bump;

    public SwerveHoming swerveHomingCommand;



    public RobotContainer() {

        sampleForward = new SampleForward();
        sampleBackward = new SampleForward();

        swerveSubsystem = new SwerveDriveSubsystem();

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

        this.y_button.onTrue(sampleForward);
        this.y_button.onFalse(new InstantCommand() {
            public void initialize() {
                sampleForward.cancel();
            }
        });

        this.a_button.onTrue(sampleBackward);
        this.a_button.onFalse(new InstantCommand() {
            public void initialize() {
                sampleBackward.cancel();
            }
        });
    }   

    public SwerveDriveSubsystem getSwerveSubsystem() {
        return this.swerveSubsystem;
    }

}
