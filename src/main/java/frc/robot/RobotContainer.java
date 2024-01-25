package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;

import frc.robot.subsystems.ShooterIntake;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.Shoot;

import frc.robot.Constants.OIConstants;

import frc.robot.Robot;


public class RobotContainer {

    // Declarations

    // // Subsystems
    private final ShooterIntake shooterIntake;

    // // Buttons
    public final XboxController driverJoystick;
    public JoystickButton buttonA;
    public JoystickButton buttonB;
    public JoystickButton buttonX;
    public JoystickButton buttonY;
    public JoystickButton l_bump;
    public JoystickButton r_bump;

    // // Commands
    public Command intake;
    public Command outtake;
    public Command shoot;

    
    public RobotContainer() {

        // Initilizations

        // // Subsystems
        shooterIntake = new ShooterIntake();

        // // Buttons
        driverJoystick = new XboxController(OIConstants.kDriverJoystickPort);
        buttonA = new JoystickButton(driverJoystick, 1);
        buttonB = new JoystickButton(driverJoystick, 2);
        buttonX = new JoystickButton(driverJoystick, 3);
        buttonY = new JoystickButton(driverJoystick, 4);
        l_bump = new JoystickButton(driverJoystick, 5);
        r_bump = new JoystickButton(driverJoystick, 6);

        // // Commands
        intake = new Intake(shooterIntake);
        outtake = new Outtake(shooterIntake);
        shoot = new Shoot(shooterIntake);

        configureButtonBindings();
    };

    private void configureButtonBindings() {
        
        // Configure Commands

        // // Intake
        l_bump.onTrue(intake);
        l_bump.onFalse(new InstantCommand() {
            @Override
            public void initialize() {
                intake.cancel();
            }
        });

        // // Outtake
        r_bump.onTrue(outtake);
        r_bump.onFalse(new InstantCommand() {
            @Override
            public void initialize() {
                outtake.cancel();
            }
        });

        
        // // Outtake
        buttonA.onTrue(shoot);
        buttonA.onFalse(new InstantCommand() {
            @Override
            public void initialize() {
                shoot.cancel();
            }
        });
    }   

}
