package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.GantryManualLower;
import frc.robot.commands.GantryManualRaise;
import frc.robot.commands.SwerveHoming;
import frc.robot.subsystems.ArmBase;
import frc.robot.subsystems.SwerveDriveSubsystem;


public class RobotContainer {

    //Subsystems
    // private final SwerveDriveSubsystem swerveSubsystem;
    private final ArmBase armBase;
    
    public final CommandXboxController driverJoystick;
    public Trigger buttonA;
    public Trigger buttonB;
    public Trigger buttonX;
    public Trigger buttonY;
    public Trigger l_bump;
    public Trigger r_bump;
    public Trigger l_trigger;
    public Trigger r_trigger;
    public double r_joy_x;
    public double r_joy_y;
    public double l_joy_x;
    public double l_joy_y;
    
    // public SwerveHoming swerveHomingCommand;
    
    //Commands
    // public Command groundPreset;
    public Command gantryManualRaise;
    public Command gantryManualLower;

    public RobotContainer() {

        //Subsystems
        armBase = new ArmBase();
        // swerveSubsystem = new SwerveDriveSubsystem();

        
        driverJoystick = new CommandXboxController(OIConstants.kDriverJoystickPort);
        buttonA = driverJoystick.a();
        buttonB = driverJoystick.b();
        buttonX = driverJoystick.x();
        buttonY = driverJoystick.y();
        l_bump = driverJoystick.leftBumper();
        r_bump = driverJoystick.rightBumper();
        l_trigger = driverJoystick.leftTrigger(OIConstants.kDriverJoystickTriggerDeadzone);
        r_trigger = driverJoystick.rightTrigger(OIConstants.kDriverJoystickTriggerDeadzone);
        r_joy_x = driverJoystick.getRightX();
        r_joy_y = driverJoystick.getRightY();
        l_joy_x = driverJoystick.getLeftX();
        l_joy_y = driverJoystick.getLeftY();
        
        //Commands
        // groundPreset = new ArmPreset(Constants.ArmBase.GroundPositionRotations, Constants.ArmPivot.PivotMotorGroundRotations);
        gantryManualRaise = new GantryManualRaise(armBase);
        gantryManualLower = new GantryManualLower(armBase);

        // swerveSubsystem.setDefaultCommand(new SwerveCommand(
        //     swerveSubsystem,
        //     () -> -driverJoystick.getLeftY(),
        //     () -> driverJoystick.getLeftX(),
        //     () -> driverJoystick.getRightX(),
        //     () -> !driverJoystick.leftTrigger(OIConstants.kDriverJoystickTriggerDeadzone)));

        configureButtonBindings();
    };

    private void configureButtonBindings() {

        buttonA.onTrue(gantryManualRaise);
        buttonA.onFalse(new InstantCommand() {
            @Override
            public void initialize() {
                gantryManualRaise.cancel();
            }
        });

        buttonB.onTrue(gantryManualLower);
        buttonB.onFalse(new InstantCommand() {
            @Override
            public void initialize() {
                gantryManualLower.cancel();
            }
        });

    }   

    // public SwerveDriveSubsystem getSwerveSubsystem() {
    //     return this.swerveSubsystem;
    // }

}
