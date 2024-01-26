package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ArmBase;
import frc.robot.subsystems.ArmPivot;

public class ArmPreset extends Command {

    public double gantryRotations;
    public double pivotRotations;
    
    public ArmPreset() {
        this.addRequirements(Robot.armBase, Robot.armPivot);
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {

    }
    

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        
    }
}