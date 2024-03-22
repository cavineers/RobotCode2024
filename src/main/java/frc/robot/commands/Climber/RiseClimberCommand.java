package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;

public class RiseClimberCommand extends Command {

    private String climberSide;

    private boolean rightLowered = false;
    private boolean leftLowered = false;

    private ClimberLeft climberLeft;
    private ClimberRight climberRight;

    public RiseClimberCommand(ClimberLeft climberLeft, ClimberRight climberRight, String side) {
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
            if (climberLeft.getLeftClimberSetPoint() >= Constants.Climber.UpperClimberMaxRotations) {
                this.leftLowered = true;
            } else {
                System.out.println("Left Rising");
                this.climberLeft.setSetpointAdd(.4);
            }

        } else if (climberSide == "right") {
            // Left climber action
            if (climberRight.getRightClimberSetPoint() >= Constants.Climber.UpperClimberMaxRotations) {
                this.leftLowered = true;
            } else {
                System.out.println("Left Rising");
                this.climberRight.setSetpointAdd(.4);
            }

        }
    }

    @Override
    public void end(boolean interrupted) {
    }
}
