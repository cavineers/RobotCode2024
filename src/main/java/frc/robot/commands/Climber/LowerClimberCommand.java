package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;
import frc.robot.subsystems.ShooterIntake;


public class LowerClimberCommand extends Command {

    private String climberSide;
    
    private boolean rightLowered = false;
    private boolean leftLowered = false;

    private ClimberLeft climberLeft;
    private ClimberRight climberRight;

    public LowerClimberCommand(ClimberLeft climberLeft, ClimberRight climberRight, String side) {
        this.climberLeft = climberLeft;
        this.climberRight = climberRight;

        if (side == "left") {
            this.addRequirements(climberLeft);
        } else {
            this.addRequirements(climberRight);
        }

        climberSide = side;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        
        if (climberSide == "left") {
            //Left climber action
            if (climberLeft.getLimitSwitch("bottom")){
                System.out.println("Left off");
                climberLeft.setLeftClimberMotorState(climberLeft.leftClimberMotorState.OFF);
                this.leftLowered = true;
            } else if (!climberLeft.getLimitSwitch("bottom")) {
                System.out.println("Left lowering");
                climberLeft.setLeftClimberMotorState(climberLeft.leftClimberMotorState.REVERSED);                
            } 

        } else if (climberSide == "right") {
            //Right climber action
             if (climberRight.getLimitSwitch("bottom")){
                System.out.println("Right off");
                climberRight.setRightClimberMotorState(climberRight.rightClimberMotorState.OFF);
                this.rightLowered = true;
            } else if (!climberRight.getLimitSwitch("bottom")) {
                System.out.println("Right lowering");
                climberRight.setRightClimberMotorState(climberRight.rightClimberMotorState.REVERSED);                
            } 
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (climberSide == "left") {
            climberLeft.setLeftClimberMotorState(ClimberLeft.LeftClimberMotorState.OFF);
        } else if (climberSide == "right") {
            climberRight.setRightClimberMotorState(ClimberRight.RightClimberMotorState.OFF);
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
