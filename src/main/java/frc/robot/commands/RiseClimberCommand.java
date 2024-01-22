package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LeftClimber;
import frc.robot.subsystems.RightClimber;

public class RiseClimberCommand extends CommandBases{

    private boolean complete;

    public RiseClimberCommand() {
        this.addRequirements(Robot.leftClimber);
    }
}
