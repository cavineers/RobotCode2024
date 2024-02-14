package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmBase;

public class GantryManualRaise extends Command {
    private boolean isDone = false;
    private double m_timestamp;
    private ArmBase armBase;

    public GantryManualRaise(ArmBase armBase) {
        this.armBase = armBase;
        this.addRequirements(armBase);
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {
        this.isDone = false;
    }

    @Override
    public void execute() {
        armBase.setBaseMotorState(armBase.baseMotorState.ON);

    }

    @Override
    public void end(boolean interrupted) {
        armBase.setBaseMotorState(armBase.baseMotorState.OFF);
    }

    /*
     * @Override public boolean isFinished() { if (Timer.getFPGATimestamp() -
     * this.m_timestamp >= 0 && Robot.m_robotContainer.joy.getRawButton(0)) {
     * this.isDone = true; }
     * 
     * return this.isDone; }
     */
}
