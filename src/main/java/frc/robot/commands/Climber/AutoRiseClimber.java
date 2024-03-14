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

            // Left climber action
            if (climberLeft.getLimitSwitch() == false) {
                this.climberLeft.setSetpointAdd(.2);
            } else {
                this.climberLeft.setLeftClimberMotorPosition(Constants.Climber.UpperClimberMaxRotations);
            }

			if (climberRight.getLimitSwitch() == false) {
                this.climberRight.setSetpointAdd(.2);
            } else {
                this.climberRight.setRightClimberMotorPosition(Constants.Climber.UpperClimberMaxRotations);
            }
    }

    @Override
    public void end(boolean interrupted) {
    }
}
