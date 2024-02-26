package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmBase;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;
import frc.robot.subsystems.ShooterIntake;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.util.Alert;
import frc.util.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SystemsCheck extends Command {

   
    
    private ArmBase armBase;
    private ArmPivot armPivot;
    private SwerveDriveSubsystem swerveSubsystem;
    private VisionSubsystem visionSubsystem;
    private ShooterIntake shooterIntake;
    private ClimberLeft climberLeft;
    private ClimberRight climberRight;
    //armBase, armPivot, climberLeft, climberRight, shooterIntake, swerveSubsystem, visionSubsystem
    public SystemsCheck(ArmBase armBase,
                        ArmPivot armPivot,
                        ClimberLeft climberLeft,
                        ClimberRight climberRight,
                        ShooterIntake shooterIntake,
                        SwerveDriveSubsystem swerveSubsystem,
                        VisionSubsystem visionSubsystem
                        ) {
      // add requirements for all subsystems
      addRequirements(armBase, armPivot, swerveSubsystem, visionSubsystem, shooterIntake, climberLeft, climberRight);
                        
      this.armBase = armBase;
      this.armPivot = armPivot;
      this.climberLeft = climberLeft;
      this.climberRight = climberRight;
      this.shooterIntake = shooterIntake;
      this.swerveSubsystem = swerveSubsystem;
      this.visionSubsystem = visionSubsystem;
    }

    private void delayThread(int ms){
      try {
        Thread.sleep(ms);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    private boolean runArmCheck(){
      boolean armBaseEncoderCheck = armBase.systemEncoderCheck();
      boolean armPivotEncoderCheck = armBase.systemEncoderCheck();

      if (!(armBaseEncoderCheck && armPivotEncoderCheck)){
  
        System.out.println("ARM Encoders are null or not connected");
        new Alert("Arm Encoders are null or not connected", AlertType.ERROR).set(true);
        return false;
      }
      // ARM DOWN
      SmartDashboard.putString("System Check Status", "MOVING: Arm Base to Min");
      armBase.setSetpoint(frc.robot.Constants.ArmBase.MinRotations);
      try {
          while (!armBase.statusPID()) {
              Thread.sleep(100); // Pause for 100 milliseconds
          }
      } catch (InterruptedException e) {
          e.printStackTrace();
      }
      SmartDashboard.putString("System Check Status", "Success");
      // ARM UP
      SmartDashboard.putString("System Check Status", "MOVING: Arm Base to Maximum");
      armBase.setSetpoint(frc.robot.Constants.ArmBase.MaxRotations);
      try {
          while (!armBase.statusPID()) {
              Thread.sleep(100); // Pause for 100 milliseconds
          }
      } catch (InterruptedException e) {
          e.printStackTrace();
      }
      SmartDashboard.putString("System Check Status", "Success");
      
      // PIVOT MOTOR TESTING
      SmartDashboard.putString("System Check Status", "MOVING: Arm Pivot to Min");
      armPivot.setSetpoint(frc.robot.Constants.ArmPivot.PivotMotorLowerRotationLimit);
      try {
          while (!armPivot.statusPID()) {
              Thread.sleep(100); // Pause for 100 milliseconds
          }
      } catch (InterruptedException e) {
          e.printStackTrace();
      }
      SmartDashboard.putString("System Check Status", "Success");

      SmartDashboard.putString("System Check Status", "MOVING: Arm Pivot to Max");
      try {
          while (!armPivot.statusPID()) {
              Thread.sleep(100); // Pause for 100 milliseconds
          }
      } catch (InterruptedException e) {
          e.printStackTrace();
      }
      SmartDashboard.putString(SystemsCheck.class.getName(), "Success");
      
      return true;


    }
    
    private boolean validateSubsytems(){
      if (this.armBase == null){
        System.out.println("Arm Base subsystem is null");
        return false;
      }
      if (this.armPivot == null){
        System.out.println("Arm Pivot subsystem is null");
        return false;
      }
      if (this.climberLeft == null){
        System.out.println("Climber Left subsystem is null");
        return false;
      }
      if (this.climberRight == null){
        System.out.println("Climber Right subsystem is null");
        return false;
      }
      if (this.shooterIntake == null){
        System.out.println("Shooter Intake subsystem is null");
        return false;
      }
      if (this.swerveSubsystem == null){
        System.out.println("Swerve Drive subsystem is null");
        return false;
      }
      if (this.visionSubsystem == null){
        System.out.println("Vision subsystem is null");
        return false;
      }
      return true;
    }
    @Override
    public void initialize() {
      SmartDashboard.putString("System Check Status", "Initializing");
      System.out.println("System Check Status: Initializing");
      delayThread(100);
      SmartDashboard.putString("System Check Status", "Validating Subsystems");
      delayThread(100);

  
      System.out.println("System Check Status: Validating Subsystems");
      delayThread(100);

      if (!validateSubsytems()){
        SmartDashboard.putString("System Check Status", "FAIL Subsystems are not valid");
        System.out.println("System Check Status: Subsystems are not valid");
        new Alert("Invalid Subsystem Initialization", AlertType.ERROR).set(true);
        return;
      }
      delayThread(100);

      SmartDashboard.putString("System Check Status", "Running Arm Check");
      
      delayThread(100);
      
      if (!runArmCheck()){
        SmartDashboard.putString("System Check Status", "FAIL Arm subsystems failed check");
        System.out.println("System Check Status: Arm subsystems failed check");
        new Alert("Arm subsystems failed check", AlertType.ERROR).set(true);
        return;
      }
      
    
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

	}
}
