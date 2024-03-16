package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;

public class FeedNote extends Command {

    private Intake intake;
    private boolean isDone = false;

    public FeedNote(Intake intake) {
        this.intake = intake;
        this.isDone = false;
        this.addRequirements(intake);
    }

    @Override
    public void execute() {
        // SmartDashboard.putString("Intake", "Feeding");

        intake.setIntakeMotorState(intake.intakeMotorState.ON);

    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeMotorState(intake.intakeMotorState.OFF);
    }


}
