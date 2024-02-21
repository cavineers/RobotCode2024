package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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


        this.armBase.setSetpoint(gantryRotations);
        this.armPivot.setSetpoint(pivotRotations);

    }

    @Override
    public void end(boolean interrupted) {

    }
}