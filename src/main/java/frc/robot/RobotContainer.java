package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.OIConstants;
import frc.robot.Robot;


public class RobotContainer {

    public Command lowerLeftClimber = new LowerClimberCommand();
    public Command riseLeftClimber = new RiseClimberCommand();

    public Command lowerRightClimber = new LowerClimberCommand();
    public Command riseRightClimber = new RiseClimberCommand();

    public JoystickButton l_bump;
    public JoystickButton r_bump;

    public RobotContainer() {
        
        driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
        l_bump = new JoystickButton(driverJoystick, 5);
        r_bump = new JoystickButton(driverJoystick, 5);
        
        lowerLeftClimber = new LowerLeftClimber("left");
        riseLeftClimber = new RiseLeftClimber("left");
        lowerRightClimber = new LowerRightClimber("right");
        riseRightClimber = new RiseRightClimber("right");

        configureButtonBindings();
    };

    private void configureButtonBindings() {
        this.l_bump.onTrue(raiseLeftClimber);
        this.r_bump.onTrue(raiseRightClimber);
    }   

}
