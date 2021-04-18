package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends RobotPart {
	private HardwareController servo = null;
	private int wobbleGrabPosition = 1;
	private boolean isRBDown = false;

	public Claw(Gamepad gp, Servo s) {
		super(gp);
		servo = new HardwareController();
		servo.addServo(s);
	}

	@Override
	public void driverUpdate() {
		if (gamepad.right_bumper) {
			if (!isRBDown) {
				//gets run only once when first pressed
				wobbleGrabPosition++;
				if (wobbleGrabPosition > 1) {
					wobbleGrabPosition = 0;
				}
			}
			isRBDown = true;
		} else {
			isRBDown = false;
		}
		servo.setPosition(wobbleGrabPosition);
	}

	@Override
	protected void autonomousUpdate() {

	}

	public void setGrab(int grab) {
		servo.setPosition(grab * .5);
	}
}