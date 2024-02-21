package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmPivot;

public class PivotManualLower extends Command {
    private boolean isDone = false;
    private double m_timestamp;
    private ArmPivot armPivot;

    public PivotManualLower(ArmPivot armPivot) {
        this.armPivot = armPivot;
        this.addRequirements(armPivot);
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {
        this.isDone = false;
    }

    @Override
    public void execute() {

            armPivot.setPivotMotorState(armPivot.pivotMotorState.REVERSED);

    }

    @Override
    public void end(boolean interrupted) {
        armPivot.setPivotMotorState(armPivot.pivotMotorState.OFF);
    }

}
