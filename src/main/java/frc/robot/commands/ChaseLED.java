package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Blinkin;

public class ChaseLED extends Command {
    
    public ChaseLED() {

        this.addRequirements(Robot.blinkin);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        
        Robot.blinkin.lightsChase();
    }

    @Override
    public void end(boolean interrupted) {
        
        Robot.blinkin.lightsChase();
    }
}