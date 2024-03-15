package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot_Auto extends Command {

    private boolean isDone = false;
    private Shooter shooter;
	private Intake intake;
    private ArmPivot armPivot;
    private VisionSubsystem visionSubsystem;

    private double requiredArmPivotAngleDegrees;
    private double currentShooterAngleFromBaseline;

    private double distance;
		
	private Timer timer;
    private Timer timer2;

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
		
		distance = visionSubsystem.getDistanceFromSpeaker();

        if (!visionSubsystem.autoShootCapable()){
            this.isDone = true;
            return;
        }

		SmartDashboard.putString("Shooter", "Auto Shooting");

		armPivot.setArmPivotAngle(calculateRequiredArmPivotAngle(distance));
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

         
        requiredArmPivotAngleDegrees = (-1.4 * (Math.pow(1.686, -(distance - 6.7)))) - 32 + 90.0;


        return requiredArmPivotAngleDegrees;
    }


    @Override
    public boolean isFinished() {
        return this.isDone;
    }

}
