package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;

public class RiseClimberCommand extends Command {

    private String climberSide;

    private boolean rightRaised = false;
    private boolean leftRaised = false;

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
    public void initialize() {

    }

    @Override
    public void execute() {

        if (climberSide == "left") {

            // Left climber action
            // Command uses a limit switch to turn the extension motor until the arm is
            // fully retracted
            if (climberLeft.getLimitSwitch("top")) {
                climberLeft.setLeftClimberMotorState(ClimberLeft.LeftClimberMotorState.OFF);
                this.leftRaised = true;
            } else if (!climberLeft.getLimitSwitch("top")) {
                System.out.println("Right raising");
                climberLeft.setLeftClimberMotorState(ClimberLeft.LeftClimberMotorState.ON);
            }

        } else if (climberSide == "right") {

            // Right climber action
            // Command uses a limit switch to turn the extension motor until the arm is
            // fully retracted
            if (climberRight.getLimitSwitch("top")) {
                climberRight.setRightClimberMotorState(ClimberRight.RightClimberMotorState.OFF);
                this.rightRaised = true;
            } else if (!climberRight.getLimitSwitch("top")) {
                System.out.println("Left raising");
                climberRight.setRightClimberMotorState(ClimberRight.RightClimberMotorState.ON);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (climberSide == "left") {
            climberLeft.setLeftClimberMotorState(ClimberLeft.LeftClimberMotorState.OFF);
        } else if (climberSide == "right") {
            climberRight.setRightClimberMotorState(ClimberRight.RightClimberMotorState.OFF);
        }
    }

    @Override
    // Once the two climbers have reached their limit of rotations, the command is
    // finished
    public boolean isFinished() {

        boolean finish = false;

        if (climberSide == "left") {
            finish = this.leftRaised;
        } else if (climberSide == "right") {
            finish = this.rightRaised;
        }
        return finish;
    }

}
