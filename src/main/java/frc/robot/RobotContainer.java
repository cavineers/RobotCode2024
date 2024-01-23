package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ChaseLED;
import frc.robot.commands.FireLED;
import frc.robot.commands.OceanLED;
import frc.robot.commands.RainbowLED;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.commands.SwerveHoming;
import frc.robot.Robot;


public class RobotContainer {

    public Command chaseLED;
    public Command fireLED;
    public Command oceanLED;
    public Command rainbowLED;

    public Joystick joy = new Joystick(0);
    public JoystickButton a_button = new JoystickButton(joy, 1);
    public JoystickButton b_button = new JoystickButton(joy, 2);
    public JoystickButton x_button = new JoystickButton(joy, 3);
    public JoystickButton y_button = new JoystickButton(joy, 4);
    public JoystickButton l_bump = new JoystickButton(joy, 5);
    public JoystickButton r_bump = new JoystickButton(joy, 6);
    public JoystickButton left_menu = new JoystickButton(joy, 7);
    public JoystickButton right_menu = new JoystickButton(joy, 8);
    public JoystickButton left_stick = new JoystickButton(joy, 9);
    public JoystickButton right_stick = new JoystickButton(joy, 10);

    public POVButton povUp = new POVButton(joy, 0, 0);
    public POVButton povRight = new POVButton(joy, 90, 0);
    public POVButton povDown = new POVButton(joy, 180, 0);
    public POVButton povLeft = new POVButton(joy, 270, 0); 

    private final SwerveDriveSubsystem swerveSubsystem;


    private final Joystick driverJoystick;
    private final JoystickButton button;

    public SwerveHoming swerveHomingCommand;



    public RobotContainer() {

        chaseLED = new ChaseLED();
        fireLED = new FireLED();
        oceanLED = new OceanLED();
        rainbowLED = new RainbowLED();

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

        this.a_button.onTrue(chaseLED);
        this.a_button.onFalse(new InstantCommand() {
            public void initialize() {
                chaseLED.cancel();
            }
        });

        this.y_button.onTrue(fireLED);
        this.y_button.onFalse(new InstantCommand() {
            public void initialize() {
                fireLED.cancel();
            }
        });

        this.x_button.onTrue(oceanLED);
        this.x_button.onFalse(new InstantCommand() {
            public void initialize() {
                oceanLED.cancel();
            }
        });

        this.b_button.onTrue(rainbowLED);
        this.b_button.onFalse(new InstantCommand() {
            public void initialize() {
                rainbowLED.cancel();
            }
        });
    }   

    public SwerveDriveSubsystem getSwerveSubsystem() {
        return this.swerveSubsystem;
    }

}
