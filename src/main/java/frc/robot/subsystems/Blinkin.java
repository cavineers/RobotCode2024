package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase {

	public static Spark blinkinLeft;
	public static Spark blinkinRight;

	//PWM port 0
	public Blinkin() {
		blinkinLeft = new Spark(0);
		blinkinRight = new Spark(1);
	}

	public void lightsRainbow() {
		blinkinLeft.set(-0.99);
		blinkinRight.set(-0.99);

	}

	public void lightsFire() {
		blinkinLeft.set(-0.59);
		blinkinRight.set(-0.59);
	}

	public void lightsChase() {
		blinkinLeft.set(0.01);
		blinkinRight.set(0.01);
	}

	public void lightsOcean() {
		blinkinLeft.set(-0.41);
		blinkinRight.set(-0.41);
	}

	public void lightsOrange() {
		blinkinLeft.set(0.65);
		blinkinRight.set(0.65);
	}
}