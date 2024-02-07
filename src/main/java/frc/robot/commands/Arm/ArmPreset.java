package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmBase;
import frc.robot.subsystems.ArmPivot;

public class ArmPreset extends Command {

    public double gantryRotations;
    public double pivotRotations;

    private ArmBase armBase;
    private ArmPivot armPivot;

    public ArmPreset(ArmBase armBase, ArmPivot armPivot, double g, double p) {
        this.addRequirements();

        this.armBase = armBase;
        this.armPivot = armPivot;

        gantryRotations = g;
        pivotRotations = p;
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {

        
    }
    
    @Override
    public void execute() {
        if(armBase.getBaseMotorPosition() >= gantryRotations + Constants.ArmBase.ArmBaseEcoderDeadzone) {
            armBase.setBaseMotorState(armBase.baseMotorState.REVERSED);
        } else if(armBase.getBaseMotorPosition() <= gantryRotations - Constants.ArmBase.ArmBaseEcoderDeadzone) {
            armBase.setBaseMotorState(armBase.baseMotorState.ON);
        } else if(armPivot.getPivotMotorPosition() >= pivotRotations + Constants.ArmPivot.ArmPivotEcoderDeadzone) {
            armPivot.setPivotMotorState(armPivot.pivotMotorState.REVERSED);
        } else if(armPivot.getPivotMotorPosition() <= pivotRotations - Constants.ArmPivot.ArmPivotEcoderDeadzone) {
            armPivot.setPivotMotorState(armPivot.pivotMotorState.ON);
        }

    }

    @Override
    public void end(boolean interrupted) {
        
    }
}