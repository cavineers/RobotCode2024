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


    private final Joystick driverJoystick;
    private final JoystickButton button;
    public JoystickButton l_bump;

    lowerLeftClimber = new LowerLeftClimber("left");
    riseLeftClimber = new RiseLeftClimber("left");
    lowerRightClimber = new LowerRightClimber("right");
    riseRightClimber = new RiseRightClimber("right");

    public RobotContainer() {
        
        driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
        button = new JoystickButton(driverJoystick, 4);
        l_bump = new JoystickButton(driverJoystick, 5);
        
        configureButtonBindings();
    };

    private void configureButtonBindings() {

    }   

}
