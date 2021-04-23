package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Extensions extends RobotPart {
	private Servo servo1, servo2 = null;
	private int extensionPosition = 1;
	private boolean isXDown = false;

	public Extensions(Gamepad gp, Servo s, Servo s2) {
		super(gp);
		servo1 = s;
		servo2 = s2;
	}

	@Override
	public void driverUpdate() {
		if (gamepad.x) {
			if (!isXDown) {
				//gets run only once when first pressed
				extensionPosition++;
				if (extensionPosition > 1) {
					extensionPosition = 0;
				}
			}
			isXDown = true;
		} else {
			isXDown = false;
		}
		servo1.setPosition(extensionPosition);
		servo2.setPosition(1 - extensionPosition);
	}

	@Override
	protected void autonomousUpdate() {

	}

	public void setExtensionPosition(double pos) {
		servo1.setPosition(pos);
		servo2.setPosition(1 - pos);
	}
}