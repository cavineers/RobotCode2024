package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.SwerveCommand;
import frc.robot.commands.SwerveHoming;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.OIConstants;

import frc.robot.Robot;
import frc.robot.subsystems.ShooterIntake;
import frc.robot.subsystems.ArmBase;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;
import frc.robot.commands.Arm.ArmPreset;
import frc.robot.commands.Arm.GantryManualLower;
import frc.robot.commands.Arm.GantryManualRaise;
import frc.robot.commands.Arm.PivotManualLower;
import frc.robot.commands.Arm.PivotManualRaise;
import frc.robot.commands.Climber.LowerClimberCommand;
import frc.robot.commands.Climber.RiseClimberCommand;
import frc.robot.commands.ShooterIntake.Intake;
import frc.robot.commands.ShooterIntake.Outtake;
import frc.robot.commands.ShooterIntake.Shoot;
import frc.robot.commands.ShooterIntake.Shoot_Manual;
import frc.robot.commands.ShooterIntake.Shoot_Auto;




public class RobotContainer {

	// Declarations
	private final ArmBase armBase;
	private final ArmPivot armPivot;

	private final SwerveDriveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;

	private final ShooterIntake shooterIntake;

	private final ClimberLeft climberLeft;
	private final ClimberRight climberRight;

	// Main Controller Buttons Init
	public final CommandXboxController xboxController0;

	// Second Controller Buttons Init
	public final CommandXboxController xboxController1;

	// Commands
	public Command gantryManualRaise;
	public Command gantryManualLower;
	public Command pivotManualRaise;
	public Command pivotManualLower;
	public Command groundPickup;
	public Command shootPosition;
	public Command sourcePosition;
	public Command ampPosition;
	public Command restPosition;

	public Command lowerLeftClimber;
	public Command riseLeftClimber;
	public Command lowerRightClimber;
	public Command riseRightClimber;

	public Command intake;
	public Command outtake;
	public Command shoot;
	public Command shootAuto;
	public Command shootManual;
	public SwerveHoming swerveHomingCommand;

	public RobotContainer() {

		// Subsystems
		armBase = new ArmBase();
		armPivot = new ArmPivot();


		climberLeft = new ClimberLeft();
		climberRight = new ClimberRight();

		shooterIntake = new ShooterIntake();

		visionSubsystem = new VisionSubsystem();
		
		swerveSubsystem = new SwerveDriveSubsystem(visionSubsystem);
		swerveHomingCommand = new SwerveHoming(swerveSubsystem);

		// First Controller
		xboxController0 = new CommandXboxController(OIConstants.kDriverJoystickPort);

		// Second Driver Buttons
		xboxController1 = new CommandXboxController(OIConstants.kSecondDriverJoystickPort);

		// Commands
		gantryManualRaise = new GantryManualRaise(armBase);
		gantryManualLower = new GantryManualLower(armBase);
		pivotManualRaise = new PivotManualRaise(armPivot);
		pivotManualLower = new PivotManualLower(armPivot);
		groundPickup = new ArmPreset(armBase, armPivot, Constants.ArmBase.GroundPickupRotations, Constants.ArmPivot.GroundPickupRotations);
		shootPosition = new ArmPreset(armBase, armPivot, Constants.ArmBase.ShootRotations, Constants.ArmPivot.ShootRotations);
		sourcePosition = new ArmPreset(armBase, armPivot, Constants.ArmBase.SourceRotations, Constants.ArmPivot.SourceRotations);
		ampPosition = new ArmPreset(armBase, armPivot, Constants.ArmBase.AmpRotations, Constants.ArmPivot.AmpRotations);
		restPosition = new ArmPreset(armBase, armPivot, Constants.ArmBase.RestRotations, Constants.ArmPivot.RestRotations);

		lowerLeftClimber = new LowerClimberCommand(climberLeft, climberRight, "left");
		riseLeftClimber = new RiseClimberCommand(climberLeft, climberRight, "left");
		lowerRightClimber = new LowerClimberCommand(climberLeft, climberRight, "right");
		riseRightClimber = new RiseClimberCommand(climberLeft, climberRight, "right");

		intake = new Intake(shooterIntake);
		outtake = new Outtake(shooterIntake);
		shoot = new Shoot(shooterIntake);
		shootManual = new Shoot_Manual(shooterIntake, () -> xboxController0.getRightTriggerAxis());
		shootAuto = new Shoot_Auto(shooterIntake, armPivot, armBase);

		swerveSubsystem.setDefaultCommand(new SwerveCommand(
					swerveSubsystem,
					() -> -xboxController0.getRawAxis(OIConstants.kDriverYAxis),
					() -> -xboxController0.getRawAxis(OIConstants.kDriverXAxis),
					() -> -xboxController0.getRawAxis(OIConstants.kDriverRotAxis),
					() -> false));
		configureButtonBindings();

	};

	private void configureButtonBindings() {

		// Arm Commands
		xboxController0.povRight().onTrue(gantryManualRaise);
		xboxController0.povRight().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				gantryManualRaise.cancel();
			}
		});

		xboxController0.povLeft().onTrue(gantryManualLower);
		xboxController0.povLeft().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				gantryManualLower.cancel();
			}
		});

		// driverJoystick.a().onTrue(pivotManualLower);
		// buttonA.onFalse(new InstantCommand() {
		// 	@Override
		// 	public void initialize() {
		// 		pivotManualLower.cancel();
		// 	}
		// });

		xboxController0.y().onTrue(pivotManualRaise);
		xboxController0.y().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				pivotManualRaise.cancel();
			}
		});

		// secondDPadDown.onTrue(groundPickup);
		// secondDPadUp.onTrue(shootPosition);
		// secondDPadLeft.onTrue(ampPosition);
		// secondDPadRight.onTrue(sourcePosition);
		// secondLeftBump.onTrue(restPosition);

		// Shooter-Intake Commands
		xboxController0.leftBumper().onTrue(outtake);
		xboxController0.leftBumper().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				outtake.cancel();
			}
		});

		xboxController0.rightBumper().onTrue(intake);
		xboxController0.rightBumper().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				intake.cancel();
			}
		});

		xboxController0.rightTrigger(OIConstants.kTriggerDeadzone).onTrue(shootManual);
		xboxController0.rightTrigger(OIConstants.kTriggerDeadzone).onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				shootManual.cancel();
			}
		});

		xboxController0.a().onTrue(shoot);
		xboxController0.a().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				shoot.cancel();
			}
		});

		// ClimberCommands
		xboxController1.a().onTrue(lowerLeftClimber);
		xboxController1.a().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				lowerLeftClimber.cancel();
			}
		});

		xboxController1.y().onTrue(riseLeftClimber);
		xboxController1.y().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				riseLeftClimber.cancel();
			}
		});

		xboxController1.x().onTrue(lowerRightClimber);
		xboxController1.x().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				lowerRightClimber.cancel();
			}
		});

		xboxController1.b().onTrue(riseRightClimber);
		xboxController1.b().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				riseRightClimber.cancel();
			}
		});

		// public SwerveDriveSubsystem getSwerveSubsystem() {
		// return this.swerveSubsystem;
		// }

		driverJoystick.a().whileTrue(shootAuto);

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
