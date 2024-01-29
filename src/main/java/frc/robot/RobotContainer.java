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
import frc.robot.commands.Arm.ArmPreset;
import frc.robot.Robot;
import frc.robot.subsystems.ArmBase;
import frc.robot.commands.Arm.Gantry.GantryManualLower;
import frc.robot.commards.Arm.Gantry.GantryManualRaise;


public class RobotContainer {

    private final SwerveDriveSubsystem swerveSubsystem;

    public Command groundPreset;


    private final Joystick driverJoystick;
    private final JoystickButton button;
    public JoystickButton l_bump;

    public SwerveHoming swerveHomingCommand;
   
    private final ArmBase armBase;
    
    //Commands
    public Command gantryManualRaise;
    public Command gantryManualLower;

    public RobotContainer() {

        armBase = new ArmBase();

        groundPreset = new ArmPreset(Constants.ArmBase.GroundPositionRotations, Constants.ArmPivot.PivotMotorGroundRotations);

        swerveSubsystem = new SwerveDriveSubsystem();

        swerveHomingCommand = new SwerveHoming(swerveSubsystem);

        driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
        button = new JoystickButton(driverJoystick, 4);
        l_bump = new JoystickButton(driverJoystick, 5);
    
        gantryManualRaise = new GantryManualRaise(armBase);
        gantryManualLower = new GantryManualLower(armBase);

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

}
