package frc.robot.commands.Climber;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;

public class ClimberAutoBalancingCommand extends Command {
	
	private AHRS gyro = new AHRS(SPI.Port.kMXP); 
	private double roll;
	private boolean isBalanced;
    private ClimberLeft climberLeft;
    private ClimberRight climberRight;

	public ClimberAutoBalancingCommand(){
		isBalanced = false;
		this.climberLeft = climberLeft;
        this.climberRight = climberRight;
		this.addRequirements(climberLeft, climberRight);
	}

	@Override
    public void execute() {
		this.roll = gyro.getRoll();

		if (climberRight.getLimitSwitch("top")) {
			climberRight.setRightClimberMotorState(climberRight.rightClimberMotorState.OFF);
			
		} else if (climberLeft.getLimitSwitch("top")) {
			climberLeft.setLeftClimberMotorState(climberLeft.leftClimberMotorState.OFF);
		
		} else if (gyro.getRoll() != 0) {
			if (roll < 0) {
				climberLeft.setLeftClimberMotorState(climberLeft.leftClimberMotorState.ON);
			} else if (roll > 0){
				climberRight.setRightClimberMotorState(climberRight.rightClimberMotorState.ON);
			} else {
				isBalanced = true;
			}
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
