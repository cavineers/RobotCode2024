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

        this.armPivot.setSetpointAdd(+.001);

    }

    @Override
    public void end(boolean interrupted) {

    }

}