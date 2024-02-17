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
    private ClimberLeft climberLeft;
    private ClimberRight climberRight;

	public ClimberAutoBalancingCommand(ClimberLeft climberLeft, ClimberRight climberRight){

		gyro = new AHRS(SPI.Port.kMXP);
		roll = gyro.getRoll();
		setpoint = 0;
		pidController = new PIDController(frc.robot.Constants.Climber.kP, frc.robot.Constants.Climber.kI, frc.robot.Constants.Climber.kD);

		isBalanced = false;
		this.climberLeft = climberLeft;
        this.climberRight = climberRight;
		this.addRequirements(climberLeft, climberRight);
	}

	@Override
    public void execute() {

		roll = gyro.getRoll();

		if (roll > 0) {
			//Apply a PID loop to the right climber
			isBalanced = false;
			climberRight.rightClimberMotor.set(pidController.calculate(roll, setpoint));

		} else if (roll < 0) {
			//Apply a PID loop to the left climber
			isBalanced = false;
			climberLeft.leftClimberMotor.set(pidController.calculate(roll, setpoint));
		} else {
			isBalanced = true;
		}
	}

	//Set tolerance 

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
