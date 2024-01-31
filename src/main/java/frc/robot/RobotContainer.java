package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.Robot;

import frc.robot.commands.LowerClimberCommand;
import frc.robot.commands.RiseClimberCommand;
import frc.robot.subsystems.LeftClimber;
import frc.robot.subsystems.RightClimber;

public class RobotContainer {

    //Climber commands
    private Command lowerLeftClimber;
    private Command riseLeftClimber;
    private Command lowerRightClimber;
    private Command riseRightClimber;

    //Controller
    private Joystick joy = new Joystick(0);
    private JoystickButton l_bump = new JoystickButton(joy,5);
    private JoystickButton r_bump = new JoystickButton(joy, 6);
    private JoystickButton x_button = new JoystickButton(joy,3);
    private JoystickButton b_button = new JoystickButton(joy, 2);


    public RobotContainer() {

        // lowerLeftClimber = new LowerClimberCommand("left");
        // riseLeftClimber = new RiseClimberCommand("left");
        // lowerRightClimber = new LowerClimberCommand("right");
        // riseRightClimber = new RiseClimberCommand("right");

        configureButtonBindings();
    };

    private void configureButtonBindings() {
        r_bump.whileTrue(new RiseClimberCommand("right"));
        l_bump.whileTrue(new RiseClimberCommand("left"));
        x_button.whileTrue(new LowerClimberCommand("left"));
        b_button.whileTrue(new LowerClimberCommand("right"));
    }
}
