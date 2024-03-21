package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.SwerveAimToTarget;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.SwerveHoming;
import frc.robot.commands.Arm.ArmPreset;
import frc.robot.commands.Arm.GantryManualLower;
import frc.robot.commands.Arm.GantryManualRaise;
import frc.robot.commands.Arm.PivotManualLower;
import frc.robot.commands.Arm.PivotManualRaise;
import frc.robot.commands.Climber.AutoLowerClimber;
import frc.robot.commands.Climber.AutoRiseClimber;
import frc.robot.commands.Climber.LowerClimberCommand;
import frc.robot.commands.Climber.RiseClimberCommand;
import frc.robot.commands.Intake.Outtake;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Intake.FeedNote;
import frc.robot.commands.Shooter.Amp;
import frc.robot.commands.Shooter.Shoot_Auto;
import frc.robot.commands.Shooter.Shoot_Manual;
import frc.robot.commands.Shooter.Shoot_Toggle;


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

	public static final Blinkin blinkin = new Blinkin();

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
	public Command shootClosePosition;
	public Command sourcePosition;
	public Command ampPosition;
	public Command restPosition;
	public Command shootGround;

	// public Command shootAuto;
	public Command lowerLeftClimber;
	public Command riseLeftClimber;
	public Command lowerRightClimber;
	public Command riseRightClimber;
	public Command autoRiseClimber;
	public Command autoLowerClimber;
	public Command armShootAutoCenterPosition;

	public Command intakeNote;
	public Command outtake;
	public Command feedNote;

	public Command shoot;
	public Command shootAuto;
	public Command shootManual;
	public Command shootToggle;
	public Command amp;
	public Command swerveAimToTarget;
	public SwerveHoming swerveHomingCommand;

	public SendableChooser<Command> autoChooser; 
	

	public RobotContainer() {

		// Subsystems
		armBase = new ArmBase();
		armPivot = new ArmPivot(armBase);

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
		shootClosePosition = new ArmPreset(armBase, armPivot, Constants.ArmBase.ShootRotations, Constants.ArmPivot.ShootCloseRotations);
		shootGround = new ArmPreset(armBase, armPivot, Constants.ArmBase.ShootGroundRotations, Constants.ArmPivot.ShootGroundRotations);
		sourcePosition = new ArmPreset(armBase, armPivot, Constants.ArmBase.SourceRotations, Constants.ArmPivot.SourceRotations);
		ampPosition = new ArmPreset(armBase, armPivot, Constants.ArmBase.AmpRotations, Constants.ArmPivot.AmpRotations);
		restPosition = new ArmPreset(armBase, armPivot, Constants.ArmBase.RestRotations, Constants.ArmPivot.RestRotations);

		// AUTO PRESETS

		armShootAutoCenterPosition = new ArmPreset(armBase, armPivot, 0, 0.4525);

		lowerLeftClimber = new LowerClimberCommand(climberLeft, climberRight, "left");
		riseLeftClimber = new RiseClimberCommand(climberLeft, climberRight, "left");
		lowerRightClimber = new LowerClimberCommand(climberLeft, climberRight, "right");
		riseRightClimber = new RiseClimberCommand(climberLeft, climberRight, "right");
		autoLowerClimber = new AutoLowerClimber(climberLeft, climberRight);
		autoRiseClimber = new AutoRiseClimber(climberLeft, climberRight);

		intakeNote = new IntakeNote(intake, shooter);
		outtake = new Outtake(intake);
		shoot = new Shoot(shooter, intake);
		shootAuto = new Shoot_Auto(shooter, intake, armPivot,visionSubsystem);
		shootManual = new Shoot_Manual(shooter, () -> xboxController0.getRightTriggerAxis());
		shootToggle = new Shoot_Toggle(shooter);
		amp = new Amp(shooter, intake);
		feedNote = new FeedNote(intake);
		


		swerveSubsystem.setDefaultCommand(new SwerveCommand(
					swerveSubsystem,
					() -> -xboxController0.getRawAxis(OIConstants.kDriverYAxis),
					() -> -xboxController0.getRawAxis(OIConstants.kDriverXAxis),
					() -> -xboxController0.getRawAxis(OIConstants.kDriverRotAxis),
					() -> !xboxController0.leftStick().getAsBoolean()));

		swerveAimToTarget = new SwerveAimToTarget(swerveSubsystem, visionSubsystem, 
					() -> -xboxController0.getRawAxis(OIConstants.kDriverYAxis),
					() -> -xboxController0.getRawAxis(OIConstants.kDriverXAxis),
					() -> !xboxController0.leftStick().getAsBoolean());

		armPivot.initializeDutyEncoder();
		armBase.initializeEncoder();
		configureButtonBindings();
		configureNamedCommands();
		
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

	};

	private void configureButtonBindings() {

		xboxController0.povUp().onTrue(new InstantCommand() {
			@Override
			public void initialize() {
				swerveSubsystem.zeroHeading();
			}
		});

		// Arm Commands
		xboxController0.povRight().whileTrue(gantryManualRaise);
		xboxController0.povLeft().whileTrue(gantryManualLower);
		xboxController0.a().whileTrue(pivotManualLower);
		xboxController0.y().whileTrue(pivotManualRaise);
		
		// Shooter-Intake Commands
		xboxController0.leftBumper().whileTrue(outtake);
		xboxController0.rightBumper().whileTrue(intakeNote);
		xboxController0.b().onTrue(shoot);
		xboxController0.x().onTrue(shootAuto);
		xboxController0.rightTrigger(Constants.OIConstants.kDriverJoystickTriggerDeadzone).whileTrue(shootManual);
		xboxController0.povDown().onTrue(amp);
		xboxController0.start().toggleOnTrue(shootToggle);
		xboxController0.back().whileTrue(feedNote);
		
		//Presets
		xboxController1.povDown().onTrue(groundPickup);
		xboxController1.povUp().onTrue(shootClosePosition);
		xboxController1.povLeft().onTrue(ampPosition);
		xboxController1.povRight().onTrue(sourcePosition);
		xboxController1.leftBumper().onTrue(restPosition);
		xboxController1.rightBumper().onTrue(shootGround);

		// Climber
		xboxController1.a().whileTrue(lowerLeftClimber);
		xboxController1.y().whileTrue(riseLeftClimber);
		xboxController1.x().whileTrue(lowerRightClimber);
		xboxController1.b().whileTrue(riseRightClimber);
		xboxController1.leftStick().onTrue(autoRiseClimber);
		xboxController1.rightStick().onTrue(autoLowerClimber);

	}
	private void configureNamedCommands(){
		NamedCommands.registerCommand("groundPickup", groundPickup);
		NamedCommands.registerCommand("shootClosePosition", shootClosePosition);
		NamedCommands.registerCommand("shootGround", shootGround);
		NamedCommands.registerCommand("sourcePosition", sourcePosition);
		NamedCommands.registerCommand("ampPosition", ampPosition);
		NamedCommands.registerCommand("restPosition", restPosition);
	    NamedCommands.registerCommand("intakeNote", intakeNote);
		NamedCommands.registerCommand("outtake", outtake);
		NamedCommands.registerCommand("feedNote", feedNote);
		NamedCommands.registerCommand("shoot", shoot);
		NamedCommands.registerCommand("shootAuto", shootAuto);
		NamedCommands.registerCommand("amp", amp);
		NamedCommands.registerCommand("shootCenterAuto", armShootAutoCenterPosition);
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
		return autoChooser.getSelected();
    }
    

	public void teleopSetup(){
		armPivot.initializeDutyEncoder();
		armBase.initializeEncoder();
	}
}
