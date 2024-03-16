package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {

    private Intake intake;
    private boolean isDone;

    public IntakeNote(Intake intake) {
        this.intake = intake;
        this.addRequirements(intake);
    }

    @Override
    public void initialize() {
        this.isDone = false;
    }

    @Override
    public void execute() {
        // SmartDashboard.putString("Intake", "Intaking");

        if (intake.getNoteSensor() == false) {
            intake.setIntakeMotorState(intake.intakeMotorState.ON);
        } else {
            this.isDone = true;
            intake.setIntakeMotorState(intake.intakeMotorState.OFF);
        }

    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeMotorState(intake.intakeMotorState.OFF);
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

}
