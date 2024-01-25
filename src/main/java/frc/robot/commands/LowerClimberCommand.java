package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LeftClimber;
import frc.robot.subsystems.RightClimber;

public class LowerClimberCommand extends Command {

    private String climberSide;

    private boolean leftLowered = false;
    private boolean rightLowered = false;

    public LowerClimberCommand(String side) {
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
            if (Robot.leftClimber.getLeftClimberMotorPosition() > Constants.Climber.ClimberLowerSpeedRotations){
                Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.REVERSED);
            } else if (Robot.leftClimber.getLeftClimberMotorPosition() < Constants.Climber.ClimberLowerSpeedRotations) {
                Robot.leftClimber.getLeftClimberMotor().set(-0.25); //TBD
            } else {
                Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
            }

            //Bottom software stop
            if (Robot.leftClimber.getLimitSwitch()){
                Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
                this.leftLowered = true;
            }
        } else if (climberSide == "right") {

             //Right climber action
            // Command uses a limit switch to turn the extension motor until the arm is fully retracted
            if (Robot.rightClimber.getRightClimberMotorPosition() > Constants.Climber.ClimberLowerSpeedRotations){
                Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.REVERSED);
            } else if (Robot.rightClimber.getRightClimberMotorPosition() < Constants.Climber.ClimberLowerSpeedRotations) {
                Robot.rightClimber.getRightClimberMotor().set(-0.25); //TBD
            } else {
                Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
            }

            //Bottom software stop
            if (Robot.rightClimber.getLimitSwitch()){
                Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
                this.rightLowered = true;
            }
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
        Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
    }

    @Override
    //Once the two climbers have been lowered, the command is finished
    public boolean isFinished() {
        return this.leftLowered && this.rightLowered;
    }
}
