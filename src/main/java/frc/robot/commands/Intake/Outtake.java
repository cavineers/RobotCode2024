package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Outtake extends Command {

    private Intake intake;

    public Outtake(Intake intake) {
        this.intake = intake;
        this.addRequirements(intake);
    }

    @Override
    public void execute() {

        SmartDashboard.putString("Intake", "Outtaking");

        intake.setIntakeMotorState(intake.intakeMotorState.REVERSE);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeMotorState(intake.intakeMotorState.OFF);
    }

}
