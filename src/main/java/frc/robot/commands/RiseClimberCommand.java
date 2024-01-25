package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LeftClimber;
import frc.robot.subsystems.RightClimber;

public class RiseClimberCommand extends Command{

    private String climberSide;
    
    private boolean rightLimitReached = false;
        private boolean leftLimitReached = false;

    public RiseClimberCommand(String side) {
        this.addRequirements(Robot.leftClimber, Robot.rightClimber);
        climberSide = side;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        if (climberSide == "left") {
            //Left climber action
            // Command uses a limit switch to turn the extension motor until the arm is fully retracted
            if (Robot.leftClimber.getLeftClimberMotorPosition() >= Constants.Climber.ClimberRiseSpeedRotations){
                Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
            } else if (Robot.leftClimber.getLeftClimberMotorPosition() < Constants.Climber.ClimberRiseSpeedRotations) {
                Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.ON);
            } else {
                Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
                this.leftLimitReached = true;
            }
        } else if (climberSide == "right") {

            //Right climber action
            // Command uses a limit switch to turn the extension motor until the arm is fully retracted
            if (Robot.rightClimber.getRightClimberMotorPosition() >= Constants.Climber.ClimberRiseSpeedRotations){
                Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
            } else if (Robot.rightClimber.getRightClimberMotorPosition() < Constants.Climber.ClimberRiseSpeedRotations) {
                Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.ON);
            } else {
                Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
                this.rightLimitReached = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
        Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
    }

    @Override
    //Once the two climbers have reached their limit of rotations, the command is finished
    public boolean isFinished() {
        return this.leftLimitReached && this.rightLimitReached;
    }

}
