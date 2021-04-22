package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Whacker extends RobotPart {
	private HardwareController whacker = null;

	public Whacker(Gamepad gp, Servo ser) {
		super(gp);
		whacker = new HardwareController();
		whacker.addServo(ser);
	}

	@Override
	protected void driverUpdate() {
		if (gamepad.dpad_up) {
			whacker.setPosition(0.35);
		} else {
			whacker.setPosition(0);
		}
	}

	@Override
	protected void autonomousUpdate() {

	}

	public void setPosition(double p) {
		whacker.setPosition(p * 0.35);
	}

	public double getPosition() {
		return whacker.getPos();
	}
}