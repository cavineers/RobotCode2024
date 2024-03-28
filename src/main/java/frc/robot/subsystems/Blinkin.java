package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase {

	public static Spark blinkin;

	//PWM port 0
	public Blinkin() {
		blinkin = new Spark(0);
	}

	public void lightsRainbow() {
		blinkin.set(-0.99);
	}

	public void lightsFire() {
		blinkin.set(-0.59);
	}

	public void lightsChase() {
		blinkin.set(0.01);
	}

	public void lightsOcean() {
		blinkin.set(-0.41);
	}

	public void lightsOrange() {
		blinkin.set(0.65);
	}

	public void lightsGreen(){
		blinkin.set(0.77);
	}
}