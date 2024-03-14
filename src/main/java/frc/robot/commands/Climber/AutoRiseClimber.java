package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;

public class AutoRiseClimber extends Command {

    private boolean rightLowered = false;
    private boolean leftLowered = false;

    private ClimberLeft climberLeft;
    private ClimberRight climberRight;

    public AutoRiseClimber(ClimberLeft climberLeft, ClimberRight climberRight) {
        this.climberLeft = climberLeft;
        this.climberRight = climberRight;

        this.addRequirements(climberLeft);
        this.addRequirements(climberRight);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

            if(climberLeft.getLimitSwitch() == false) {
				climberLeft.setLeftClimberMotorSpeed(0.05);
			} else {
				climberLeft.setLeftClimberMotorPosition(Constants.Climber.UpperClimberMaxRotations);
                climberLeft.setSetpoint(Constants.Climber.UpperClimberMaxRotations);
			}

			if(climberRight.getLimitSwitch() == false) {
				climberRight.setRightClimberMotorSpeed(0.05);
			} else {
				climberRight.setRightClimberMotorPosition(Constants.Climber.UpperClimberMaxRotations);
                climberRight.setSetpoint(Constants.Climber.UpperClimberMaxRotations);
			}
        }
    

    @Override
    public void end(boolean interrupted) {
    }
}
