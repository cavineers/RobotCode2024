package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.SampleSystem;

public class SampleBackward extends Command {
    
    public SampleBackward() {

        this.addRequirements(Robot.sampleSystem);
    }

    @Override
    public void initialize() {

        Robot.sampleSystem.setSampleMotorState(SampleSystem.MotorState.REVERSED);
    }

    @Override
    public void execute() {
        
        Robot.sampleSystem.setSampleMotorState(SampleSystem.MotorState.OFF);
    }

    @Override
    public void end(boolean interrupted) {
        
        Robot.sampleSystem.setSampleMotorState(SampleSystem.MotorState.OFF);
    }
}
