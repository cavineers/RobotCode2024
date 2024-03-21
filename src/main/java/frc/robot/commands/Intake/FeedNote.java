package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class FeedNote extends Command {

    private Intake intake;
    private boolean isDone = false;
    private Timer timer;

    public FeedNote(Intake intake) {
        this.intake = intake;
        this.isDone = false;
        this.addRequirements(intake);

        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        this.isDone = false;
    }

    @Override
    public void execute() {
        // SmartDashboard.putString("Intake", "Feeding");

        intake.setIntakeMotorState(intake.intakeMotorState.ON);

        if(timer.get()>.5) {
            this.isDone = true;
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
