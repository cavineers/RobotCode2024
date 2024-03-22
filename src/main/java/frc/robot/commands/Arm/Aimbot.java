package frc.robot.commands.Arm;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.VisionSubsystem;

public class Aimbot extends Command {

    private boolean isDone = false;
    private ArmPivot armPivot;

    private double requiredArmPivotAngleDegrees;
    private double currentShooterAngleFromBaseline;

    private double distanceInches;

    private VisionSubsystem visionSubsystem;

    private ShuffleboardTab robot = Shuffleboard.getTab("Robot");

    private GenericEntry distanceInchesSliderEntry = robot
        .add("Distance Slider", 12)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 12, "max", 150))
        .getEntry();
    
    private GenericEntry distanceInchesDoubleEntry = robot
        .add("Distance", 12.0)
        .getEntry();
		
    public Aimbot(ArmPivot armPivot, VisionSubsystem visionSubsystem) {
        this.armPivot = armPivot;
        this.visionSubsystem = visionSubsystem;
        this.addRequirements(armPivot);
    }

    // Set Motor State to ON / OFF
    @Override
    public void initialize() {
		this.isDone = false;
    }

    @Override
    public void execute() {
		
		distanceInches = distanceInchesSliderEntry.getDouble(12);

		armPivot.setArmPivotAngle(calculateRequiredArmPivotAngle(distanceInches));
        
    }

    @Override
    public void end(boolean interrupted) {
        this.isDone = true;
        // SmartDashboard.putString("Shooter", "Done Auto Shooting");
    }

    public double calculateRequiredArmPivotAngle(Double distance) {

		SmartDashboard.putNumber("Distance to Speaker", distance);
         
        requiredArmPivotAngleDegrees = 78.9 * (Math.pow(Math.sin(((Math.PI * 0.503 * distanceInches) + 3.9) / 180), .5)) -116.67;

        distanceInchesDoubleEntry.setDouble(requiredArmPivotAngleDegrees);
        SmartDashboard.putNumber("Required Arm Angle", requiredArmPivotAngleDegrees);

        return requiredArmPivotAngleDegrees;
    }


    @Override
    public boolean isFinished() {
        return this.isDone;
    }

}
