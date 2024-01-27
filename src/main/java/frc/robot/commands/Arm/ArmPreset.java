package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ArmBase;
import frc.robot.subsystems.ArmPivot;

public class ArmPreset extends Command {

    public double gantryRotations;
    public double pivotRotations;

    public ArmPreset(double g, double p) {
        this.addRequirements(Robot.armBase, Robot.armPivot);

        gantryRotations = g;
        pivotRotations = p;
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {

        if(Robot.armBase.getBaseMotorPosition() >= gantryRotations + Constants.ArmBase.ArmBaseEcoderDeadzone) {
            Robot.armBase.setBaseMotorState(ArmBase.BaseMotorState.REVERSED);
        } else if(Robot.armBase.getBaseMotorPosition() <= gantryRotations - Constants.ArmBase.ArmBaseEcoderDeadzone) {
            Robot.armBase.setBaseMotorState(ArmBase.BaseMotorState.ON);
        } else if(Robot.armPivot.getPivotMotorPosition() >= pivotRotations + Constants.ArmPivot.ArmPivotEcoderDeadzone) {
            Robot.armPivot.setPivotMotorState(ArmPivot.PivotMotorState.REVERSED);
        } else if(Robot.armPivot.getPivotMotorPosition() <= pivotRotations - Constants.ArmPivot.ArmPivotEcoderDeadzone) {
            Robot.armPivot.setPivotMotorState(ArmPivot.PivotMotorState.ON);
        }
    }
    
    @Override
    public void execute() {
 
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}