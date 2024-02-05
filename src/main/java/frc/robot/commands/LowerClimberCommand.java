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
            if (Robot.leftClimber.getLimitSwitch("bottom")){
                System.out.println("Left off");
                Robot.leftClimber.setLeftClimberMotorState(Robot.leftClimber.leftClimberMotorState.OFF);
                this.leftLowered = true;
            } else if (!Robot.leftClimber.getLimitSwitch("bottom")) {
                System.out.println("Left lowering");
                Robot.leftClimber.setLeftClimberMotorState(Robot.leftClimber.leftClimberMotorState.REVERSED);                
            } 

        } else if (climberSide == "right") {
            //Right climber action
             if (Robot.rightClimber.getLimitSwitch("bottom")){
                System.out.println("Right off");
                Robot.rightClimber.setRightClimberMotorState(Robot.rightClimber.rightClimberMotorState.OFF);
                this.rightLowered = true;
            } else if (!Robot.rightClimber.getLimitSwitch("bottom")) {
                System.out.println("Right lowering");
                Robot.rightClimber.setRightClimberMotorState(Robot.rightClimber.rightClimberMotorState.REVERSED);                
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
            finish = this.leftLowered;
        } else if (climberSide == "right") {
            finish = this.rightLowered;
        }
        return finish;
    }
}
