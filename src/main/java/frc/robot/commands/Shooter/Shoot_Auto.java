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

    private double distanceMeters;

    private VisionSubsystem visionSubsystem;
		
	private Timer timer;
    private Timer timer2;

    private InterpolatingDoubleTreeMap interpolatePivotAngleMap;
    private InterpolatingDoubleTreeMap interpolateShooterSpeedMap;

 

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
        this.interpolateShooterSpeedMap = new InterpolatingDoubleTreeMap();

        initMaps();

        
    }

    private void initMaps(){
        // PIVOT ANGLE MAP
        interpolatePivotAngleMap.put(0.0, 0.0);
        interpolatePivotAngleMap.put(1.0, 1.0);
        interpolatePivotAngleMap.put(2.0, 2.0);
        interpolatePivotAngleMap.put(3.0, 3.0);
        interpolatePivotAngleMap.put(4.0, 4.0);
        interpolatePivotAngleMap.put(5.0, 5.0);
        interpolatePivotAngleMap.put(6.0, 6.0);
        interpolatePivotAngleMap.put(7.0, 7.0);
        interpolatePivotAngleMap.put(8.0, 8.0);
        interpolatePivotAngleMap.put(9.0, 9.0);

        // SHOOTER SPEED MAP

        interpolateShooterSpeedMap.put(0.0, 0.0);
        interpolateShooterSpeedMap.put(1.0, 1.0);
        interpolateShooterSpeedMap.put(2.0, 2.0);
        interpolateShooterSpeedMap.put(3.0, 3.0);
        interpolateShooterSpeedMap.put(4.0, 4.0);
        interpolateShooterSpeedMap.put(5.0, 5.0);
        interpolateShooterSpeedMap.put(6.0, 6.0);
        interpolateShooterSpeedMap.put(7.0, 7.0);
        interpolateShooterSpeedMap.put(8.0, 8.0);


    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {
		timer.reset();
        timer2.reset();
        timer.start();

		this.isDone = false;
    }

    @Override
    public void execute() {
		
		distanceMeters = visionSubsystem.getDistanceFromSpeaker();

		// SmartDashboard.putString("Shooter", "Auto Shooting");

		armPivot.setArmPivotAngle(calculateRequiredArmPivotAngle(distanceMeters));
        shooter.setShooterMotorState(shooter.shooterMotorState.ON);
        if (armPivot.isAtSetpoint() && timer.get()>interpolateShooterSpeedMap.get(distanceMeters)){
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

        // requiredArmPivotAngleDegrees = (-1.4 * (Math.pow(1.686, -(distance - 6.7)))) - 29.4 + 90.0;

        requiredArmPivotAngleDegrees = interpolatePivotAngleMap.get(distance);

        SmartDashboard.putNumber("Required Arm Angle", requiredArmPivotAngleDegrees);

        return requiredArmPivotAngleDegrees;
    }


    @Override
    public boolean isFinished() {
        return this.isDone;
    }

}
