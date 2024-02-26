package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SystemsCheck extends Command {

    public SystemsCheck(Subsystem... subsystems) {
    	// add requirements for all subsystems
		for (Subsystem subsystem : subsystems)
			this.addRequirements(subsystem);
    }


    @Override
    public void initialize() {
		// cancel all commands other than this one
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

	}
}
