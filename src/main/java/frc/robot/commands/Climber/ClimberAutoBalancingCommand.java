package frc.robot.commands.Climber;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;

public class ClimberAutoBalancingCommand extends Command {
	
	private AHRS gyro; 
	private PIDController pidController;
	private double roll;
	private double setpoint;

	private boolean isBalanced;
    private ClimberLeft climberLeft;
    private ClimberRight climberRight;
	private int threshold;

	public ClimberAutoBalancingCommand(ClimberLeft climberLeft, ClimberRight climberRight){

		gyro = new AHRS(SPI.Port.kMXP);
		roll = gyro.getRoll();
		setpoint = 0;
		pidController = new PIDController(frc.robot.Constants.Climber.kP, frc.robot.Constants.Climber.kI, frc.robot.Constants.Climber.kD);

		isBalanced = false;
		this.climberLeft = climberLeft;
        this.climberRight = climberRight;

		threshold = 2;
		this.addRequirements(climberLeft, climberRight);
	}

	@Override
    public void execute() {
		
		roll = gyro.getRoll() + 2.48;
		double u = (pidController.calculate(roll, setpoint))/100;

		SmartDashboard.putNumber("Roll", roll);
		SmartDashboard.putNumber("U", u);
		SmartDashboard.putNumber("Left motor speed", climberLeft.getLeftClimberMotorSpeed());
		SmartDashboard.putNumber("Right motor speed", climberRight.getRightClimberMotorSpeed());

		if (roll > threshold) {
			climberRight.rightClimberMotor.set(u);
			climberLeft.setLeftClimberMotorState(climberLeft.leftClimberMotorState.OFF);

		} else if (roll < -threshold) {
			climberLeft.leftClimberMotor.set(Math.abs(u));
			climberRight.setRightClimberMotorState(climberRight.rightClimberMotorState.OFF);

		} else {
			climberLeft.setLeftClimberMotorState(climberLeft.leftClimberMotorState.OFF);
			climberRight.setRightClimberMotorState(climberRight.rightClimberMotorState.OFF);
		}

		// System.out.println("balance!");
		// roll = gyro.getRoll();
		// double u = pidController.calculate(roll, setpoint);

		// if (u > 0 && !climberRight.getLimitSwitch("top")) {
		// 	climberRight.rightClimberMotor.set(u);

		// } else if (u < 0 && !climberLeft.getLimitSwitch("top")) {
		// 	climberLeft.leftClimberMotor.set(Math.abs(u));

		// } else if (climberRight.getLimitSwitch("top")){
		// 	climberRight.setRightClimberMotorState(ClimberRight.RightClimberMotorState.OFF);
		// } else if (climberLeft.getLimitSwitch("top")) {
		// 	climberLeft.setLeftClimberMotorState(ClimberLeft.LeftClimberMotorState.OFF);
		// }

		// if (roll > 0 && !climberRight.getLimitSwitch("top")) {
		// 	//Apply a PID loop to the right climber
		// 	isBalanced = false;
		// 	climberRight.rightClimberMotor.set(pidController.calculate(roll, setpoint));

		// } else if (roll < 0 && !climberLeft.getLimitSwitch("top")) {
		// 	//Apply a PID loop to the left climber
		// 	isBalanced = false;
		// 	climberLeft.leftClimberMotor.set(pidController.calculate(roll, setpoint));
		// } else {
		// 	isBalanced = true;
		// }
	}

	//Set tolerance 
	//@Override
    //public void end(boolean interrupted) {
		//climberLeft.setLeftClimberMotorState(ClimberLeft.LeftClimberMotorState.OFF);
		//climberRight.setRightClimberMotorState(ClimberRight.RightClimberMotorState.OFF);
	//}

	//@Override
    // Once the robot is balanced, end the command.
    // public boolean isFinished() {
	// 	return isBalanced;
	// }

}
