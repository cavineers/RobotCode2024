package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LeftClimber;
import frc.robot.subsystems.RightClimber;

public class LowerClimberCommand extends CommandBase {

    private String climberSide;

    public LowerClimberCommand(String side) {
        this.addRequirements(Robot.LeftClimber, Robot.RightClimber);
        climberSide = side;
    }

    @Override
    public void initialize() {
        private boolean this.leftLowered = false;
        private boolean this.rightLowered = false;
    }

    @Override
    public void execute() {

        if (climberSide == "left") {

             //Left climber action
            // Command uses a limit switch to turn the extension motor until the arm is fully retracted
            if (Robot.LeftClimber.getLeftClimberMotorPosition() > Constants.Climber.ClimberLowerSpeedRotations){
                Robot.LeftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.REVERSED);
            } else if (Robot.LeftClimber.getLeftClimberMotorPosition() < Constants.Climber.ClimberLowerSpeedRotations) {
                Robot.LeftClimber.getLeftClimberMotor().set(-0.25); //TBD
            } else {
                Robot.LeftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
            }

            //Bottom software stop
            if (Robot.LeftClimber.getLimitSwitch()){
                Robot.LeftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
                this.leftlowered = true;
            }
        } else if (climberSide == "right") {

             //Right climber action
            // Command uses a limit switch to turn the extension motor until the arm is fully retracted
            if (Robot.RightClimber.getRightClimberMotorPosition() > Constants.Climber.ClimberLowerSpeedRotations){
                Robot.RightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.REVERSED);
            } else if (Robot.RightClimber.getRightClimberMotorPosition() < Constants.Climber.ClimberLowerSpeedRotations) {
                Robot.RightClimber.getRightClimberMotor().set(-0.25); //TBD
            } else {
                Robot.RightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
            }

            //Bottom software stop
            if (Robot.RightClimber.getLimitSwitch()){
                Robot.RightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
                this.rightLowered = true;
            }
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        Robot.LeftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
        Robot.RightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
    }

    @Override
    //Once the two climbers have been lowered, the command is finished
    public boolean isFinished() {
        return this.leftLowered && this.rightLowered;
    }
}
