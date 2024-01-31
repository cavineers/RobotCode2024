package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SampleSystem;

public class AmperageCommand extends Command {
    
    public AmperageCommand() {

        this.addRequirements(Robot.sampleSystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        
        if(Robot.sampleSystem.getSampleMotorCurrent() > Constants.SampleSystem.SampleMotorMaxAmperage) {
            cancel();
        } else {
            Robot.sampleSystem.setSampleMotorState(SampleSystem.MotorState.ON);
        }

    }

    @Override
    public void end(boolean interrupted) {
        
        Robot.sampleSystem.setSampleMotorState(SampleSystem.MotorState.OFF);
    }
}
