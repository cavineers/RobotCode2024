package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OIConstants;

import frc.robot.Robot;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ArmBase;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Shooter;

import frc.robot.commands.SwerveCommand;
import frc.robot.commands.SwerveHoming;
import frc.robot.commands.Arm.ArmPreset;
import frc.robot.commands.Arm.GantryManualLower;
import frc.robot.commands.Arm.GantryManualRaise;
import frc.robot.commands.Arm.PivotManualLower;
import frc.robot.commands.Arm.PivotManualRaise;
import frc.robot.commands.Climber.LowerClimberCommand;
import frc.robot.commands.Climber.RiseClimberCommand;
import frc.robot.commands.Intake.Outtake;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.Amp;
import frc.robot.commands.Shooter.Shoot_Manual;

public class RobotContainer {

	// Declarations
	private final ArmBase armBase;
	private final ArmPivot armPivot;

	private final SwerveDriveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;

	private final Intake intake;
	private final Shooter shooter;

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

	public Command shootAuto;

	public Command lowerLeftClimber;
	public Command riseLeftClimber;
	public Command lowerRightClimber;
	public Command riseRightClimber;

	public Command intakeNote;
	public Command outtake;

	public Command shoot;
	public Command amp;
	public Command shootManual;
	public SwerveHoming swerveHomingCommand;

	public RobotContainer() {

		// Subsystems
		armBase = new ArmBase();
		armPivot = new ArmPivot();

		climberLeft = new ClimberLeft();
		climberRight = new ClimberRight();

		intake = new Intake();
		shooter = new Shooter();

		visionSubsystem = new VisionSubsystem();
		
		swerveSubsystem = new SwerveDriveSubsystem(visionSubsystem);
		swerveHomingCommand = new SwerveHoming(swerveSubsystem);

		// Controllers
		xboxController0 = new CommandXboxController(OIConstants.kDriverJoystickPort);
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

		intakeNote = new IntakeNote(intake);
		outtake = new Outtake(intake);
		shoot = new Shoot(shooter, intake);
		amp = new Amp(shooter);

		// shootAuto = new Shoot_Auto(shooter, armPivot, armBase);

		shootManual = new Shoot_Manual(shooter, () -> xboxController0.getRightTriggerAxis());

		swerveSubsystem.setDefaultCommand(new SwerveCommand(
					swerveSubsystem,
					() -> -xboxController0.getRawAxis(OIConstants.kDriverYAxis),
					() -> -xboxController0.getRawAxis(OIConstants.kDriverXAxis),
					() -> -xboxController0.getRawAxis(OIConstants.kDriverRotAxis),
					() -> false));

		armPivot.initializeDutyEncoder();
		armBase.initializeEncoder();
		configureButtonBindings();

	};

	private void configureButtonBindings() {

		xboxController0.povUp().onTrue(new InstantCommand() {
			@Override
			public void initialize() {
				swerveSubsystem.zeroHeading();
			}
		});

		xboxController0.povDown().onTrue(amp);
		xboxController0.povDown().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				amp.cancel();
			}
		});

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

		xboxController0.a().onTrue(pivotManualLower);
		xboxController0.a().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				pivotManualLower.cancel();
			}
		});

		xboxController0.y().onTrue(pivotManualRaise);
		xboxController0.y().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				pivotManualRaise.cancel();
			}
		});

		xboxController0.x().onTrue(shootAuto);

		// Shooter-Intake Commands
		xboxController0.leftBumper().onTrue(outtake);
		xboxController0.leftBumper().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				outtake.cancel();
			}
		});

		xboxController0.rightBumper().onTrue(intakeNote);
		xboxController0.rightBumper().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				intakeNote.cancel();
			}
		});

		xboxController0.b().onTrue(shoot);
		xboxController0.b().onFalse(new InstantCommand() {
			@Override
			public void initialize() {
				shoot.cancel();
			}
		});

		xboxController1.povDown().onTrue(groundPickup);
		xboxController1.povUp().onTrue(shootPosition);
		xboxController1.povLeft().onTrue(ampPosition);
		xboxController1.povRight().onTrue(sourcePosition);
		xboxController1.leftBumper().onTrue(restPosition);

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

	}

	public SwerveDriveSubsystem getSwerveSubsystem() {
        return this.swerveSubsystem;
    }

    public VisionSubsystem getVisionSubsystem() {
        return this.visionSubsystem;
    }

	public ArmBase getArmBase() {
		return this.armBase;
	}

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("TestAutoNow");
    }
}
