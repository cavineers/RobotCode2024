package frc.robot.commands.Climber;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;

public class ClimberAutoBalancingCommand extends Command {
	
	private AHRS gyro; 
	private PIDController pidController;
	private double roll;
	private double setpoint;

	private boolean isBalanced;
	private String side;
    private ClimberLeft climberLeft;
    private ClimberRight climberRight;

	public ClimberAutoBalancingCommand(ClimberLeft climberLeft, ClimberRight climberRight, String side){

		gyro = new AHRS(SPI.Port.kMXP);
		roll = gyro.getRoll();
		setpoint = 0;
		PIDController pidController = new PIDController(frc.robot.Constants.Climber.kP, frc.robot.Constants.Climber.kI, frc.robot.Constants.Climber.kD);

		isBalanced = false;
		this.climberLeft = climberLeft;
        this.climberRight = climberRight;

		if (side == "left") {
			this.addRequirements(climberLeft);
		} else {
			this.addRequirements(climberLeft);
		}
	}

	@Override
    public void execute() {

		roll = gyro.getRoll();

		if (roll > 0) {
			//Apply a PID loop to the right climber
			isBalanced = false;
			climberRight.rightClimberMotor.set(pidController.calculate(climberRight.rightClimberMotor.getEncoder().getDistance(), setpoint));

		} else if (roll < 0) {
			//Apply a PID loop to the left climber
			isBalanced = false;
			climberLeft.leftClimberMotor.set(pidController.calculate(climberLeft.leftClimberMotor.getEncoder().getDistance(), setpoint));
		} else {
			isBalanced = true;
		}
	}

	@Override
    public void end(boolean interrupted) {
		climberLeft.setLeftClimberMotorState(ClimberLeft.LeftClimberMotorState.OFF);
		climberRight.setRightClimberMotorState(ClimberRight.RightClimberMotorState.OFF);
	}

	@Override
    // Once the robot is balanced, end the command.
    public boolean isFinished() {
		return isBalanced;
	}

}
