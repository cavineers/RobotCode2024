package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ArmBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ManualRaise extends Command {
    
    private boolean isDone = false;
    ArmBase base;

    public ManualRaise(ArmBase base) {
        this.base = base;
        this.addRequirements(Robot.armBase);
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {
        this.isDone = false;
    }

    @Override
    public void execute() {
        if(base.getBaseMotorPosition() < Constants.ArmBase.MaxRotations) {
            Robot.armBase.setBaseMotorState(ArmBase.BaseMotorState.ON);
        } else {
            Robot.armBase.setBaseMotorState(ArmBase.BaseMotorState.OFF);
            this.isDone = true;
        }
}

    @Override
    public void end(boolean interrupted) {
        base.setBaseMotorState(ArmBase.BaseMotorState.OFF);
    }

    @Override
    public boolean isFinished() {
        return this.isDone;
    }
}