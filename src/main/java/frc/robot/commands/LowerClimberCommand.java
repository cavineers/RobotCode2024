package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;


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
    public void execute() {

        
        if (climberSide == "left") {
            //Left climber action
            //If the number of rotations has exceeded the lower rotation limit, turn the motors off. Otherwise, continue lowering
            if (Robot.leftClimber.getLeftClimberMotorPosition() <= Constants.Climber.LowerClimberMinRotations){
                Robot.leftClimber.setLeftClimberMotorState(Robot.leftClimber.leftClimberMotorState.OFF);
                this.leftLowered = true;
            } else {
                Robot.leftClimber.setLeftClimberMotorState(Robot.leftClimber.leftClimberMotorState.REVERSED);                
            } 

        } else if (climberSide == "right") {
            //Right climber action
            //If the number of rotations has exceeded the lower rotation limit, turn the motors off. Otherwise, continue lowering
             if (Robot.rightClimber.getRightClimberMotorPosition() <= Constants.Climber.LowerClimberMinRotations){
                Robot.rightClimber.setRightClimberMotorState(Robot.rightClimber.rightClimberMotorState.OFF);
                this.rightLowered = true;
            } else {
                Robot.rightClimber.setRightClimberMotorState(Robot.rightClimber.rightClimberMotorState.REVERSED);                
            } 
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (climberSide == "left") {
            Robot.leftClimber.setLeftClimberMotorState(ClimberLeft.LeftClimberMotorState.OFF);
        } else if (climberSide == "right") {
            Robot.rightClimber.setRightClimberMotorState(ClimberRight.RightClimberMotorState.OFF);
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
}
