package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.LowerClimberCommand;
import frc.robot.commands.RiseClimberCommand;
import frc.robot.Robot;


public class RobotContainer {


    private final Joystick driverJoystick;
    private final JoystickButton button;
    public JoystickButton l_bump;

    public RobotContainer() {

        driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
        button = new JoystickButton(driverJoystick, 4);
        l_bump = new JoystickButton(driverJoystick, 5);
        
        configureButtonBindings();
    };

    private void configureButtonBindings() {

    }   

}
