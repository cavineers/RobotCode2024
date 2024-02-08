package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ShooterIntake;
import frc.robot.subsystems.ArmBase;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.Shoot;
import frc.robot.commands.Shoot_Manual;
import frc.robot.commands.Arm.GantryManualLower;
import frc.robot.commands.Arm.GantryManualRaise;
import frc.robot.commands.Arm.PivotManualLower;
import frc.robot.commands.Arm.PivotManualRaise;
import frc.robot.commands.LowerClimberCommand;
import frc.robot.commands.RiseClimberCommand;

public class RobotContainer {

	// Declarations
	private final ArmBase armBase;
	private final ArmPivot armPivot;

	private final ShooterIntake shooterIntake;

	private final ClimberLeft climberLeft;
	private final ClimberRight climberRight;

	// private final SwerveDriveSubsystem swerveSubsystem;

	// Main Controller Buttons Init
	public final CommandXboxController driverJoystick;
	public Trigger buttonA;
	public Trigger buttonB;
	public Trigger buttonX;
	public Trigger buttonY;
	public Trigger dPadRight;
	public Trigger dPadLeft;
	public Trigger dPadUp;
	public Trigger dPadDown;
	public Trigger leftBump;
	public Trigger rightBump;
	public Trigger leftTrigger;
	public Trigger rightTrigger;

	public double r_joy_x;
	public double r_joy_y;
	public double l_joy_x;
	public double l_joy_y;

	// Second Controller Buttons Init
	public final CommandXboxController secondDriverJoystick;
	public Trigger secondButtonA;
	public Trigger secondButtonB;
	public Trigger secondButtonX;
	public Trigger secondButtonY;
	public Trigger secondDPadRight;
	public Trigger secondDPadLeft;
	public Trigger secondDPadUp;
	public Trigger secondDPadDown;
	public Trigger secondLeftBump;
	public Trigger secondRightBump;

	// Commands
	public Command gantryManualRaise;
	public Command gantryManualLower;
	public Command pivotManualRaise;
	public Command pivotManualLower;

	public Command lowerLeftClimber;
	public Command riseLeftClimber;
	public Command lowerRightClimber;
	public Command riseRightClimber;

	public Command intake;
	public Command outtake;
	public Command shoot;
	public Command shootManual; 
	// public SwerveHoming swerveHomingCommand;

	public RobotContainer() {

		// Subsystems
		armBase = new ArmBase();
		armPivot = new ArmPivot();

		climberLeft = new ClimberLeft();
		climberRight = new ClimberRight();

		shooterIntake = new ShooterIntake();
		// swerveSubsystem = new SwerveDriveSubsystem();

		// First Driver Buttons
		driverJoystick = new CommandXboxController(OIConstants.kDriverJoystickPort);
		buttonA = driverJoystick.a();
		buttonB = driverJoystick.b();
		buttonX = driverJoystick.x();
		buttonY = driverJoystick.y();
		dPadRight = driverJoystick.povRight();
		dPadLeft = driverJoystick.povLeft();
		dPadUp = driverJoystick.povUp();
		dPadDown = driverJoystick.povDown();
		leftBump = driverJoystick.leftBumper();
		rightBump = driverJoystick.rightBumper();
		leftTrigger = driverJoystick.leftTrigger(OIConstants.kTriggerDeadzone);
		rightTrigger = driverJoystick.rightTrigger(OIConstants.kTriggerDeadzone);

		r_joy_x = driverJoystick.getRightX();
		r_joy_y = driverJoystick.getRightY();
		l_joy_x = driverJoystick.getLeftX();
		l_joy_y = driverJoystick.getLeftY();

		// Second Driver Buttons
		secondDriverJoystick = new CommandXboxController(OIConstants.kSecondDriverJoystickPort);
		secondButtonA = secondDriverJoystick.a();
		secondButtonB = secondDriverJoystick.b();
		secondButtonX = secondDriverJoystick.x();
		secondButtonY = secondDriverJoystick.y();
		secondDPadRight = secondDriverJoystick.povRight();
		secondDPadLeft = secondDriverJoystick.povLeft();
		secondDPadUp = secondDriverJoystick.povUp();
		secondDPadDown = secondDriverJoystick.povDown();
		secondLeftBump = secondDriverJoystick.leftBumper();
		secondRightBump = secondDriverJoystick.rightBumper();

		// Commands
		gantryManualRaise = new GantryManualRaise(armBase);
		gantryManualLower = new GantryManualLower(armBase);
		pivotManualRaise = new PivotManualRaise(armPivot);
		pivotManualLower = new PivotManualLower(armPivot);

		lowerLeftClimber = new LowerClimberCommand("left");
		riseLeftClimber = new RiseClimberCommand("left");
		lowerRightClimber = new LowerClimberCommand("right");
		riseRightClimber = new RiseClimberCommand("right"); 

		intake = new Intake(shooterIntake);
		outtake = new Outtake(shooterIntake);
		shoot = new Shoot(shooterIntake);
		shootManual = new Shoot_Manual(shooterIntake, () -> driverJoystick.getRightTriggerAxis());

		// swerveSubsystem.setDefaultCommand(new SwerveCommand(
		// swerveSubsystem,
		// () -> -driverJoystick.getLeftY(),
		// () -> driverJoystick.getLeftX(),
		// () -> driverJoystick.getRightX(),
		// () ->
		// !driverJoystick.leftTrigger(OIConstants.kDriverJoystickTriggerDeadzone)));

		configureButtonBindings();

	};

	private void configureButtonBindings() {

		// Arm Commands
		dPadRight.onTrue(gantryManualRaise);
		dPadRight.onFalse(new InstantCommand(){
			@Override
			public void initialize(){
				gantryManualRaise.cancel();
			}
		});

		dPadLeft.onTrue(gantryManualLower);
		dPadLeft.onFalse(new InstantCommand(){
			@Override
			public void initialize(){
				gantryManualLower.cancel();
			}
		});

		buttonA.onTrue(pivotManualLower);
		buttonA.onFalse(new InstantCommand(){
			@Override
			public void initialize(){
				pivotManualLower.cancel();
			}
		});

		buttonY.onTrue(pivotManualRaise);
		buttonY.onFalse(new InstantCommand(){
			@Override
			public void initialize(){
				pivotManualRaise.cancel();
			}
		});

		// Shooter-Intake Commands
		leftBump.onTrue(outtake);
		leftBump.onFalse(new InstantCommand(){
			@Override
			public void initialize(){
				outtake.cancel();
			}
		});

		rightBump.onTrue(intake);
		rightBump.onFalse(new InstantCommand(){
			@Override
			public void initialize(){
				intake.cancel();
			}
		});

		rightTrigger.onTrue(shootManual);
		rightTrigger.onFalse(new InstantCommand(){
			@Override
			public void initialize(){
				shootManual.cancel();
			}
		});

		secondRightBump.onTrue(shoot);
		secondRightBump.onFalse(new InstantCommand(){
			@Override
			public void initialize(){
				shoot.cancel();
			}
		});

		// ClimberCommands

		secondButtonA.onTrue(lowerLeftClimber);
		secondButtonA.onFalse(new InstantCommand(){
			@Override
			public void initialize(){
				lowerLeftClimber.cancel();
			}
		});

		secondButtonY.onTrue(riseLeftClimber);
		secondButtonY.onFalse(new InstantCommand(){
			@Override
			public void initialize(){
				riseLeftClimber.cancel();
			}
		});

		secondButtonX.onTrue(lowerRightClimber);
		secondButtonX.onFalse(new InstantCommand(){
			@Override
			public void initialize(){
				lowerRightClimber.cancel();
			}
		});

		secondButtonB.onTrue(riseRightClimber);
		secondButtonB.onFalse(new InstantCommand(){
			@Override
			public void initialize(){
				riseRightClimber.cancel();
			}
		});

		// public SwerveDriveSubsystem getSwerveSubsystem() {
		// return this.swerveSubsystem;
		// }

	}

}
