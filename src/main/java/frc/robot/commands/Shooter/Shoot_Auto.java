package frc.robot.commands.Shooter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot_Auto extends Command {

    private boolean isDone = false;
    private Shooter shooter;
	private Intake intake;
    private ArmPivot armPivot;

    private double requiredArmPivotAngleDegrees;
    private double currentShooterAngleFromBaseline;

    private double distanceMeters;

    ShuffleboardTab tab = Shuffleboard.getTab("Robot");

    private GenericEntry distanceEntry = tab
        .add("Distance to target (Meters)", 0)
        .getEntry();
		
	private Timer timer;
    private Timer timer2;

    public Shoot_Auto(Shooter shooter, Intake intake, ArmPivot armPivot) {
        this.shooter = shooter;
        this.intake = intake;
        this.armPivot = armPivot;
        this.addRequirements(shooter);
        this.addRequirements(intake);
        this.addRequirements(armPivot);

		timer = new Timer();
        timer2 = new Timer();
        
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
		
		distanceMeters = distanceEntry.getDouble(1);

		SmartDashboard.putString("Shooter", "Auto Shooting");

		armPivot.setArmPivotAngle(calculateRequiredArmPivotAngle(distanceMeters));
        shooter.setShooterMotorState(shooter.shooterMotorState.ON);
        if (armPivot.isAtSetpoint() && timer.get()>2) {
            SmartDashboard.putBoolean("Is At Setpoint", true);
            intake.setIntakeMotorState(intake.intakeMotorState.ON);
        }

        if (intake.getNoteSensor()== false || timer.get()>5) {
           timer2.start();
           if (timer2.get()>1){
                this.isDone = true;
           }
        }

        SmartDashboard.putNumber("Timer1", timer.get());
        SmartDashboard.putNumber("Timer2", timer2.get());
        
    }

    @Override
    public void end(boolean interrupted) {
		shooter.setShooterMotorState(shooter.shooterMotorState.OFF);
        intake.setIntakeMotorState(intake.intakeMotorState.OFF);
        timer.stop();
        SmartDashboard.putString("Shooter", "Done Auto Shooting");
    }

    public double calculateCurrentShooterAngle() {

        currentShooterAngleFromBaseline = 180 - (Constants.ArmPivot.armPivotJointAngleDegrees + armPivot.getArmPivotAngle());

        return currentShooterAngleFromBaseline;
    }

    public double calculateRequiredArmPivotAngle(Double distance) {

		SmartDashboard.putNumber("Distance to Speaker", distance);
         
        requiredArmPivotAngleDegrees = -1.7 * Math.pow(1.638, -(distance - 7.47)) - 31.65 + 90.0;

        SmartDashboard.putNumber("Required Arm Angle", requiredArmPivotAngleDegrees);

        return requiredArmPivotAngleDegrees;
    }


    @Override
    public boolean isFinished() {
        return this.isDone;
    }

}
