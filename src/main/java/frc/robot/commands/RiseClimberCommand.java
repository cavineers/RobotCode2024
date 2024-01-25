package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LeftClimber;
import frc.robot.subsystems.RightClimber;

public class RiseClimberCommand extends Command{

    private String climberSide;
    
    private boolean rightRaised = false;
    private boolean leftRaise = false;

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
            if (Robot.leftClimber.getLimitSwitch()){
                Robot.leftClimber.setLeftClimberMotorState(leftClimber.LeftClimberMotorState.OFF);
                this.leftRaised = true;
            } else if (!Robot.leftClimber.getLimitSwitch()) {
                Robot.leftClimber.setLeftClimberMotorState(leftClimber.LeftClimberMotorState.ON);
            } 

        } else if (climberSide == "right") {
            //Right climber action
            // Command uses a limit switch to turn the extension motor until the arm is fully retracted
            if (Robot.rightClimber.getLimitSwitch()){
                Robot.rightClimber.setRightClimberMotorState(rightClimber.RightClimberMotorState.OFF);
                this.rightRaised = true;
            } else if (!Robot.rightClimber.getLimitSwitch()) {
                Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.ON);
            } 
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.leftClimber.setLeftClimberMotorState(leftClimber.LeftClimberMotorState.OFF);
        Robot.rightClimber.setRightClimberMotorState(rightClimber.RightClimberMotorState.OFF);
    }

    @Override
    //Once the two climbers have reached their limit of rotations, the command is finished
    public boolean isFinished() {
        return this.leftRaised && this.rightRaised;
    }

}
