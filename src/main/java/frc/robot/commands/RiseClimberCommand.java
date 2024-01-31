package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LeftClimber;
import frc.robot.subsystems.RightClimber;

public class RiseClimberCommand extends Command{

    private String climberSide;
    
    private boolean rightRaised = false;
    private boolean leftRaised = false;

    public RiseClimberCommand(String side) {
     
        if (side == "left") {
            this.addRequirements(Robot.leftClimber);
        } else {
            this.addRequirements(Robot.rightClimber);
        }

        //this.addRequirements(Robot.leftClimber, Robot.rightClimber);
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
                Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
                this.leftRaised = true;
            } else if (!Robot.leftClimber.getLimitSwitch()) {
                Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.ON);                
            } 
            
        } else if (climberSide == "right") {
            
            //Right climber action
            // Command uses a limit switch to turn the extension motor until the arm is fully retracted
            if (Robot.rightClimber.getLimitSwitch()){
                Robot.rightClimber.setRightClimberMotorState(Robot.rightClimber.rightClimberMotorState.OFF);
                this.rightRaised = true;
            } else {
                Robot.rightClimber.setRightClimberMotorState(Robot.rightClimber.rightClimberMotorState.ON);
            } 
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (climberSide == "left") {
            Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
        } else if (climberSide == "right") {
            Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
        }
    }

    @Override
    //Once the two climbers have reached their limit of rotations, the command is finished
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
