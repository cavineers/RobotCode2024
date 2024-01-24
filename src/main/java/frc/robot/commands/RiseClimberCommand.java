package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LeftClimber;
import frc.robot.subsystems.RightClimber;

public class RiseClimberCommand extends CommandBases{

    public RiseClimberCommand() {
        this.addRequirements(Robot.LeftClimber, Robot.RightClimber);
    }

    @Override
    public void initialize() {
        private boolean this.rightLimitReached = false;
        private boolean this.leftLimitReached = false;
    }

    @Override
    public void execute() {
        //Left climber action

        // Command uses a limit switch to turn the extension motor until the arm is fully retracted
        if (Robot.LeftClimber.getLeftClimberMotorPosition() >= Constants.Climber.ClimberRiseSpeedRotations){
            Robot.LeftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
        } else if (Robot.LeftClimber.getLeftClimberMotorPosition() < Constants.Climber.ClimberRiseSpeedRotations) {
            Robot.LeftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.ON);
        } else {
            Robot.LeftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
            this.leftLimitReached = true;
        }

        //Right climber action

        // Command uses a limit switch to turn the extension motor until the arm is fully retracted
        if (Robot.RightClimber.getRightClimberMotorPosition() >= Constants.Climber.ClimberRiseSpeedRotations){
            Robot.RightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
        } else if (Robot.RihtClimber.getRightClimberMotorPosition() < Constants.Climber.ClimberRiseSpeedRotations) {
            Robot.RightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.ON);
        } else {
            Robot.RightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
            this.rightLimitReached = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.LeftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
        Robot.RightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
    }

    @Override
    //Once the two climbers have reached their limit of rotations, the command is finished
    public boolean isFinished() {
        return this.leftLimitReached && this.rightLimitReached;
    }

}
