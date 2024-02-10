package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.ArmPivot;

public class PivotManualRaise extends Command {
    private boolean isDone = false;
    private double m_timestamp;
    private ArmPivot armPivot;

    public PivotManualRaise(ArmPivot armPivot) {
        this.armPivot = armPivot;
        this.addRequirements(armPivot);
    }

    @Override
    public void initialize() {
        this.isDone = false;
    }

    @Override
    public void execute() {
        if (armPivot.getPivotEncoderPosition() < Constants.ArmPivot.PivotMotorUpperRotationLimit) {
            armPivot.setPivotMotorState(armPivot.pivotMotorState.ON);
            SmartDashboard.putString("Pivot State", "Raising");
        } else {
            armPivot.setPivotMotorState(armPivot.pivotMotorState.OFF);
            SmartDashboard.putString("Pivot State", "Hit Limit");

        }
    }

    @Override
    public void end(boolean interrupted) {
        armPivot.setPivotMotorState(armPivot.pivotMotorState.OFF);
    }

}