package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SampleSystem;

public class AmperageCommand extends Command {

    public boolean waited = false;
    
    public AmperageCommand() {

        this.addRequirements(Robot.sampleSystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        
        if(waited = true) {
            if(Robot.sampleSystem.getSampleMotorCurrent() > Constants.SampleSystem.SampleMotorMaxAmperage) {
                cancel();
            } else {
                Robot.sampleSystem.setSampleMotorState(SampleSystem.MotorState.ON);
            }
        } else {
            Robot.sampleSystem.setSampleMotorState(SampleSystem.MotorState.ON);
            
            try {
            // Wait for 1 second (1000 milliseconds)
            Thread.sleep(50);

            waited = true;
            

        } catch (InterruptedException e) {
            // Handle the exception if necessary
            e.printStackTrace();
        }
        }

        

    }

    @Override
    public void end(boolean interrupted) {
        
        Robot.sampleSystem.setSampleMotorState(SampleSystem.MotorState.OFF);
    }
}
