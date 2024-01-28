package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import frc.robot.subsystems.ShooterIntake;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.Shoot;
import frc.robot.commands.Shoot_Manual;


import frc.robot.Constants.OIConstants;

import frc.robot.Robot;


public class RobotContainer {

    // Declarations

    // // Subsystems
    private final ShooterIntake shooterIntake;

    // // Buttons
    public final CommandXboxController driverJoystick;
    public Trigger buttonA;
    public Trigger buttonB;
    public Trigger buttonX;
    public Trigger buttonY;
    public Trigger l_bump;
    public Trigger r_bump;
    public Trigger lt;
    public Trigger rt;

    // // Commands
    public Command intake;
    public Command outtake;
    public Command shoot;
    public Command shoot_manual;

    
    public RobotContainer() {

        // Initilizations

        // // Subsystems
        shooterIntake = new ShooterIntake();

        // // Buttons
        driverJoystick = new CommandXboxController(OIConstants.kDriverJoystickPort);
        buttonA = driverJoystick.a();
        buttonB = driverJoystick.b();
        buttonX = driverJoystick.x();
        buttonY = driverJoystick.y();
        l_bump = driverJoystick.leftBumper();
        r_bump = driverJoystick.rightBumper();
        lt = driverJoystick.leftTrigger(OIConstants.kTriggerDeadzone);
        rt = driverJoystick.rightTrigger(OIConstants.kTriggerDeadzone);

        // // Commands
        intake = new Intake(shooterIntake);
        outtake = new Outtake(shooterIntake);
        shoot = new Shoot(shooterIntake);
        shoot_manual = new Shoot_Manual(shooterIntake, () -> driverJoystick.getRightTriggerAxis());

        configureButtonBindings();

    };

    private void configureButtonBindings() {
        
        // Configure Commands

        // // Intake
        buttonX.onTrue(intake);
        buttonX.onFalse(new InstantCommand() {
            @Override
            public void initialize() {
                intake.cancel();
            }
        });

        // // Outtake
        buttonB.onTrue(outtake);
        buttonB.onFalse(new InstantCommand() {
            @Override
            public void initialize() {
                outtake.cancel();
            }
        });

        // // Shoot
        buttonA.onTrue(shoot);
        buttonA.onFalse(new InstantCommand() {
            @Override
            public void initialize() {
                shoot.cancel();
            }
        });

        // // Shoot Manual
        rt.onTrue(shoot_manual);
        buttonA.onFalse(new InstantCommand() {
            @Override
            public void initialize() {
                shoot_manual.cancel();
            }
        });


    }   

}
