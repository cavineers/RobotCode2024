package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OIConstants;
// import frc.robot.commands.SwerveHoming;
import frc.robot.commands.Arm.ArmPreset;
import frc.robot.commands.Arm.GantryManualLower;
import frc.robot.commands.Arm.GantryManualRaise;
import frc.robot.commands.Arm.PivotManualLower;
import frc.robot.commands.Arm.PivotManualRaise;
import frc.robot.subsystems.ArmBase;
import frc.robot.subsystems.ArmPivot;
// import frc.robot.subsystems.SwerveDriveSubsystem;


import frc.robot.subsystems.ShooterIntake;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.Shoot;
import frc.robot.commands.Shoot_Manual;
import frc.robot.Constants.OIConstants;

public class RobotContainer {

    // Declarations
  //Subsystems
    // private final SwerveDriveSubsystem swerveSubsystem;
    private final ArmBase armBase;
    private final ArmPivot armPivot;
    // // Subsystems
    private final ShooterIntake shooterIntake;

    // // Buttons
    public final CommandXboxController driverJoystick;
    public Trigger buttonA;
    public Trigger buttonB;
    public Trigger buttonX;
    public Trigger buttonY;
    public Trigger leftBump;
    public Trigger rightBump;
    public Trigger leftTrigger;
    public Trigger rightTrigger;

    public double r_joy_x;
    public double r_joy_y;
    public double l_joy_x;
    public double l_joy_y;
    
    // public SwerveHoming swerveHomingCommand;
    
    //Commands
    // public Command groundPreset;
    public Command gantryManualRaise;
    public Command gantryManualLower;
    public Command pivotManualRaise;
    public Command pivotManualLower;
    public Command armGroundPickupPreset;


    // // Commands
    public Command intake;
    public Command outtake;
    public Command shoot;
    public Command shoot_manual;

    
    public RobotContainer() {

        //Subsystems
        armBase = new ArmBase();
        armPivot = new ArmPivot();
        // swerveSubsystem = new SwerveDriveSubsystem();

        shooterIntake = new ShooterIntake();

        // // Buttons

        driverJoystick = new CommandXboxController(OIConstants.kDriverJoystickPort);
        buttonA = driverJoystick.a();
        buttonB = driverJoystick.b();
        buttonX = driverJoystick.x();
        buttonY = driverJoystick.y();
        leftBump = driverJoystick.leftBumper();
        rightBump = driverJoystick.rightBumper();

        r_joy_x = driverJoystick.getRightX();
        r_joy_y = driverJoystick.getRightY();
        l_joy_x = driverJoystick.getLeftX();
        l_joy_y = driverJoystick.getLeftY();
        
        //Commands
        // groundPreset = new ArmPreset(Constants.ArmBase.GroundPositionRotations, Constants.ArmPivot.PivotMotorGroundRotations);
        gantryManualRaise = new GantryManualRaise(armBase);
        gantryManualLower = new GantryManualLower(armBase);
        pivotManualRaise = new PivotManualRaise(armPivot);
        pivotManualLower = new PivotManualLower(armPivot);
        armGroundPickupPreset = new ArmPreset(armBase, armPivot, Constants.ArmBase.GroundPickupRotations, Constants.ArmPivot.GroundPickupRotations);


        // swerveSubsystem.setDefaultCommand(new SwerveCommand(
        //     swerveSubsystem,
        //     () -> -driverJoystick.getLeftY(),
        //     () -> driverJoystick.getLeftX(),
        //     () -> driverJoystick.getRightX(),
        //     () -> !driverJoystick.leftTrigger(OIConstants.kDriverJoystickTriggerDeadzone)));

        leftTrigger = driverJoystick.leftTrigger(OIConstants.kTriggerDeadzone);
        rightTrigger = driverJoystick.rightTrigger(OIConstants.kTriggerDeadzone);

        // // Commands
        intake = new Intake(shooterIntake);
        outtake = new Outtake(shooterIntake);
        shoot = new Shoot(shooterIntake);
        shoot_manual = new Shoot_Manual(shooterIntake, () -> driverJoystick.getRightTriggerAxis());


        configureButtonBindings();

    };

    private void configureButtonBindings() {

//         buttonX.onTrue(pivotManualRaise);
//         buttonX.onFalse(new InstantCommand(){
//         @Override
//         public void initialize(){
//             pivotManualRaise.cancel();
//         }
//     });
        
//         buttonY.onTrue(pivotManualLower);
//         buttonY.onFalse(new InstantCommand() {
//             @Override
//             public void initialize() {
//                 pivotManualLower.cancel();
//             }
//         });


//         buttonA.onTrue(gantryManualRaise);
//         buttonA.onFalse(new InstantCommand() {
//             @Override
//             public void initialize() {
//                 gantryManualRaise.cancel();
//             }
//         });

//         buttonB.onTrue(gantryManualLower);
//         buttonB.onFalse(new InstantCommand() {
//             @Override
//             public void initialize() {
//                 gantryManualLower.cancel();
//             }
//         });

//         leftBump.onTrue(armGroundPickupPreset);
//         leftBump.onFalse(new InstantCommand() {
//             @Override
//             public void initialize() {
//                 armGroundPickupPreset.cancel();
        
//         // Configure Commands

//         // // Intake
//         buttonX.onTrue(intake);
//         buttonX.onFalse(new InstantCommand() {
//             @Override
//             public void initialize() {
//                 intake.cancel();
//             }
//         });

//         // // Outtake
//         buttonB.onTrue(outtake);
//         buttonB.onFalse(new InstantCommand() {
//             @Override
//             public void initialize() {
//                 outtake.cancel();
//             }
//         });

//         // // Shoot
//         buttonA.onTrue(shoot);
//         buttonA.onFalse(new InstantCommand() {
//             @Override
//             public void initialize() {
//                 shoot.cancel();
//             }
//         });

//         // // Shoot Manual
//         rightTrigger.onTrue(shoot_manual);
//         rightTrigger.onFalse(new InstantCommand() {
//             @Override
//             public void initialize() {
//                 shoot_manual.cancel();

//             }
//         });



    // public SwerveDriveSubsystem getSwerveSubsystem() {
    //     return this.swerveSubsystem;
    // }

    }   


}
