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
            // Left climber action
            if (climberLeft.getLimitSwitch()) {
                this.leftLowered = true;
            } else if (!climberLeft.getLimitSwitch()) {
                System.out.println("Left lowering");
                this.climberLeft.setSetpointAdd(-.1);
                
            }

        } else if (climberSide == "right") {
            // Left climber action
            if (climberRight.getLimitSwitch()) {
                this.rightLowered = true;
            } else if (!climberRight.getLimitSwitch()) {
                System.out.println("Right lowering");
                this.climberRight.setSetpointAdd(-.1);
                
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
    }
}
