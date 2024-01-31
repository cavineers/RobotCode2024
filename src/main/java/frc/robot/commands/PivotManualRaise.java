package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
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
        armPivot.setPivotMotorState(armPivot.pivotMotorState.ON);
}

    @Override
    public void end(boolean interrupted) {
        armPivot.setPivotMotorState(armPivot.pivotMotorState.OFF);
    }

}