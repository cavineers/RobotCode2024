package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;

public class LowerClimberCommand extends Command {

    private String climberSide;

    private boolean rightLowered = false;
    private boolean leftLowered = false;

    private ClimberLeft climberLeft;
    private ClimberRight climberRight;

    public LowerClimberCommand(ClimberLeft climberLeft, ClimberRight climberRight, String side) {
        this.climberLeft = climberLeft;
        this.climberRight = climberRight;

        if (side == "left") {
            this.addRequirements(climberLeft);
        } else {
            this.addRequirements(climberRight);
        }

        climberSide = side;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        if (climberSide == "left") {
            this.climberLeft.setSetpointAdd(-.5);

        } else if (climberSide == "right") {

            this.climberRight.setSetpointAdd(-.5);
                
        }
        }
    

    @Override
    public void end(boolean interrupted) {
    }
}
