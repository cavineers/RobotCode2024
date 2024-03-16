package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;

public class AutoLowerClimber extends Command {

    private boolean rightLowered = false;
    private boolean leftLowered = false;

    private ClimberLeft climberLeft;
    private ClimberRight climberRight;

    public AutoLowerClimber(ClimberLeft climberLeft, ClimberRight climberRight) {
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
            climberLeft.setSetpoint(Constants.Climber.LowerClimberMaxRotations + 20);
			climberRight.setSetpoint(Constants.Climber.LowerClimberMaxRotations + 20);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
