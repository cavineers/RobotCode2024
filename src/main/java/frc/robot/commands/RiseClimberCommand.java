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
        this.addRequirements(Robot.leftClimber, Robot.rightClimber);
        climberSide = side;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        System.out.println("Rising climber");

        if (climberSide == "left") {

            System.out.println("Left climber");
            //Left climber action
            // Command uses a limit switch to turn the extension motor until the arm is fully retracted
            if (Robot.leftClimber.getLimitSwitch()){
                Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.OFF);
                this.leftRaised = true;
            } else if (!Robot.leftClimber.getLimitSwitch()) {
                Robot.leftClimber.setLeftClimberMotorState(LeftClimber.LeftClimberMotorState.ON);
            } 

        } else if (climberSide == "right") {

            System.out.println("right climber");
            //Right climber action
            // Command uses a limit switch to turn the extension motor until the arm is fully retracted
            if (Robot.rightClimber.getLimitSwitch()){
                Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.OFF);
                this.rightRaised = true;
            } else if (!Robot.rightClimber.getLimitSwitch()) {
                Robot.rightClimber.setRightClimberMotorState(RightClimber.RightClimberMotorState.ON);
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
