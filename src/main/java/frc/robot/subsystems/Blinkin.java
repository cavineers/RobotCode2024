package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase {
	
	Spark blinkin;

	public Blinkin() {
		blinkin = new Spark(0);
	}

	public void lightsRed() {
		blinkin.set(0.61);
	}

	public void lightsWhite() {
		blinkin.set(0.69);
	}
}