package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.SampleSystem;

public class SampleForward extends Command {
    
    public SampleForward() {

        this.addRequirements(Robot.sampleSystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        
        Robot.sampleSystem.setSampleMotorState(SampleSystem.MotorState.ON);
    }

    @Override
    public void end(boolean interrupted) {
        
        Robot.sampleSystem.setSampleMotorState(SampleSystem.MotorState.OFF);
    }
}
