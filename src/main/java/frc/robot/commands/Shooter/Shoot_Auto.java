package frc.robot.commands.Shooter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.ObjectInputStream.GetField;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Shoot_Auto extends Command {

    private boolean isDone = false;
    private Shooter shooter;
	private Intake intake;
    private ArmPivot armPivot;

    private double requiredArmPivotAngleDegrees;
    private double currentShooterAngleFromBaseline;

    private double distanceInches;

    private VisionSubsystem visionSubsystem;
		
	private Timer timer;
    private Timer timer2;

    private InterpolatingDoubleTreeMap interpolatePivotAngleMap;

 

    public Shoot_Auto(Shooter shooter, Intake intake, ArmPivot armPivot, VisionSubsystem visionSubsystem) {
        this.shooter = shooter;
        this.intake = intake;
        this.armPivot = armPivot;
        this.visionSubsystem = visionSubsystem;
        this.addRequirements(shooter);
        this.addRequirements(intake);
        this.addRequirements(armPivot);

		timer = new Timer();
        timer2 = new Timer();

        this.interpolatePivotAngleMap = new InterpolatingDoubleTreeMap();

        initMaps();

        
    }

    private void initMaps(){
        // PIVOT ANGLE MAP
        interpolatePivotAngleMap.put(56.82, 0.425);
        interpolatePivotAngleMap.put(62.4, 0.437);
        interpolatePivotAngleMap.put(73.25, 0.447);
        interpolatePivotAngleMap.put(85.08, 0.457);
        interpolatePivotAngleMap.put(95.48, 0.46);
        interpolatePivotAngleMap.put(105.94, 0.47);
        interpolatePivotAngleMap.put(122.36, 0.475);
        
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {
		timer.reset();
        timer2.reset();
        timer.start();

		this.isDone = false;
    }

    private boolean atPivotGoalSetpoint(double rots){
        if (Math.abs(this.armPivot.getPivotAbsolute() - rots) < 0.03){
            System.out.println("DONE");
            return true;
        }
        return false;
    }

    @Override
    public void execute() {
		
		distanceInches = visionSubsystem.getDistanceFromSpeaker();

        SmartDashboard.putNumber("DISTANCE TO AUTO SHOT", distanceInches);

		// SmartDashboard.putString("Shooter", "Auto Shooting");
        SmartDashboard.putNumber("AUTO SHOOT PIVOT SETPOINT", interpolatePivotAngleMap.get(distanceInches));
		armPivot.setArmPivotAngle(interpolatePivotAngleMap.get(distanceInches));
        armPivot.setSetpoint(interpolatePivotAngleMap.get(distanceInches));
        shooter.setShooterMotorState(shooter.shooterMotorState.ON);
        if (this.atPivotGoalSetpoint(interpolatePivotAngleMap.get(distanceInches)) && timer.get()>1.2){
            SmartDashboard.putBoolean("Is At Setpoint", true);
            intake.setIntakeMotorState(intake.intakeMotorState.ON);
        }

        if (intake.getNoteSensor()== false || timer.get()>5) {
           timer2.start();
           if (timer2.get()>1){
                this.isDone = true;
           }
        }

        // SmartDashboard.putNumber("Timer1", timer.get());
        // SmartDashboard.putNumber("Timer2", timer2.get());
        
    }

    @Override
    public void end(boolean interrupted) {
		shooter.setShooterMotorState(shooter.shooterMotorState.OFF);
        intake.setIntakeMotorState(intake.intakeMotorState.OFF);
        timer.stop();
        // SmartDashboard.putString("Shooter", "Done Auto Shooting");
    }

    public double calculateCurrentShooterAngle() {

        currentShooterAngleFromBaseline = 180 - (Constants.ArmPivot.armPivotJointAngleDegrees + armPivot.getArmPivotAngle());

        return currentShooterAngleFromBaseline;
    }

    public double calculateRequiredArmPivotAngle(Double distance) {

        requiredArmPivotAngleDegrees = 78.9 * (Math.pow(Math.sin(Math.toRadians((0.503 * distanceInches) + 1.7)), 0.5)) - 115 + 90;

        //requiredArmPivotAngleDegrees = interpolatePivotAngleMap.get(distance);

        SmartDashboard.putNumber("Required Arm Angle", requiredArmPivotAngleDegrees);

        return requiredArmPivotAngleDegrees;
    }


    @Override
    public boolean isFinished() {
        return this.isDone;
    }

}
