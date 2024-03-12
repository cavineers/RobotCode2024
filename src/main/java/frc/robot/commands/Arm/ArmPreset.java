package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        this.addRequirements(armBase, armPivot);

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

        System.out.println("Gantry Rotations: " + gantryRotations);
        System.out.println("Pivot Rotations: " + pivotRotations);
    }

    @Override
    public void execute() {


        this.armBase.setSetpoint(gantryRotations);
        this.armPivot.setSetpoint(pivotRotations);

    }

    private boolean atPivotGoalSetpoint(){
        if (Math.abs(this.armPivot.getPivotAbsolute() - this.pivotRotations) < Constants.ArmPivot.PivotSetpointTolerance){
            return true;
        }
        System.out.println("CURRENT PIVOT: " + this.armPivot.getPivotAbsolute() + "\n CURRENT STATE: " + (this.armPivot.getPivotAbsolute() - this.pivotRotations));
        return false;
    }

    private boolean atGantryGoalSetpoint(){
        if (Math.abs(this.armBase.getBaseMotorPosition() - this.gantryRotations) < 0.1){
            System.out.println(this.armPivot.getPivotAbsolute() - this.pivotRotations);
            return true;
        }
        System.out.println("CURRENT PIVOT: " + this.armPivot.getPivotAbsolute() + "\n CURRENT STATE: " + (this.armPivot.getPivotAbsolute() - this.pivotRotations));
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted){
            // Ensure the arm is at the correct position
            this.armBase.setSetpoint(gantryRotations);
            this.armPivot.setSetpoint(pivotRotations);
        }
    }

    
    @Override 
    public boolean isFinished() {
        return this.atGantryGoalSetpoint() && this.atPivotGoalSetpoint();
    }
}