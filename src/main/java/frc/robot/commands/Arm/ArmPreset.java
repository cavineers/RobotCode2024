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

    public boolean gantryDone;
    public boolean pivotDone;

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

        gantryDone = false;
        pivotDone = false;
    }

    @Override
    public void execute() {


        if (armBase.getBaseMotorPosition() >= gantryRotations + Constants.ArmBase.ArmBaseEcoderDeadzone) {
            armBase.setBaseMotorState(armBase.baseMotorState.REVERSED);
        } else if (armBase.getBaseMotorPosition() <= gantryRotations - Constants.ArmBase.ArmBaseEcoderDeadzone) {
            armBase.setBaseMotorState(armBase.baseMotorState.ON);
        } else {
            armBase.setBaseMotorState(armBase.baseMotorState.OFF);
            gantryDone = true;
        }
        
        if (armPivot.getPivotMotorPosition() >= pivotRotations + Constants.ArmPivot.ArmPivotEcoderDeadzone) {
            armPivot.setPivotMotorState(armPivot.pivotMotorState.REVERSED);
        } else if (armPivot.getPivotMotorPosition() <= pivotRotations - Constants.ArmPivot.ArmPivotEcoderDeadzone) {
            armPivot.setPivotMotorState(armPivot.pivotMotorState.ON);
        } else {
            armPivot.setPivotMotorState(armPivot.pivotMotorState.OFF);
            pivotDone = true;
        }
        
        if (gantryDone == true && pivotDone == true) {
            cancel();
        }

    }

    @Override
    public void end(boolean interrupted) {

        armBase.setBaseMotorState(armBase.baseMotorState.OFF);
        armPivot.setPivotMotorState(armPivot.pivotMotorState.OFF);
    }
}