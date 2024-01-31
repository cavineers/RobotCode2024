package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LeftClimber;
import frc.robot.subsystems.RightClimber;

public class LowerClimberCommand extends Command {

    private String climberSide;
    
    private boolean rightLowered = false;
    private boolean leftLowered = false;

    public LowerClimberCommand(String side) {
     
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
            Robot.leftClimber.setLeftClimberMotorState(Robot.leftClimber.leftClimberMotorState.REVERSED);  
        } else if (climberSide == "right") {
            //Right climber action
            Robot.rightClimber.setRightClimberMotorState(Robot.rightClimber.rightClimberMotorState.REVERSED);
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
            finish = this.leftLowered;
        } else if (climberSide == "right") {
            finish = this.rightLowered;
        }
        return finish;
    }

    /*private String climberSide;

    private boolean leftRotationsReached = false;
    private boolean rightRotationsReached = false;

    public LowerClimberCommand(String side) {
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
             //Stop the robot once it has winded down a maximum number of rotations
            if (Robot.leftClimber.getLeftClimberMotorPosition() > Constants.Climber.LowerClimberMaxRotations){
                Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.REVERSED);
            } else if (Robot.leftClimber.getLeftClimberMotorPosition() <= Constants.Climber.LowerClimberMaxRotations) {
                Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
            } 
            
        } else if (climberSide == "right") {

            //Right climber action
            //Stop the robot once it has winded down a maximum number of rotations
            if (Robot.rightClimber.getRightClimberMotorPosition() > Constants.Climber.LowerClimberMaxRotations){
                Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.REVERSED);
            } else if (Robot.rightClimber.getRightClimberMotorPosition() <= Constants.Climber.LowerClimberMaxRotations) {
                Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
            } 

        }
    }
    
    @Override
    public void end(boolean interrupted) {

        if (climberSide == "left") {
            Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
        } else {
            Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
        }
    }

    @Override
    //Once the two climbers have been lowered, the command is finished
    public boolean isFinished() {

        boolean finish = false;

        if (climberSide == "left") {
            finish = this.leftRotationsReached;
        } else {
            finish = this.rightRotationsReached;
        }
        return finish;
    }*/
}
