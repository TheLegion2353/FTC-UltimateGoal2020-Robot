package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.PID;
import org.firstinspires.ftc.teamcode.Robot.HardwareController;

public class Arm extends RobotPart {
	public double armP = 1;
	public double armI = 0.01;
	public double armD = 3;

	private int wobbleArmPositionSetpoints = 0;
	private boolean isLBDown = false;
	protected HardwareController motor = null;
	protected PID PIDController = null;
	protected Telemetry telemetry = null;
	protected double position = 1.85;

	public Arm(Gamepad gp, HardwareController hwC) {
		super(gp);
		motor = hwC;
		PIDController = new PID(armP, armI, armD, motor.getPos());
	}

	public Arm(Gamepad gp, HardwareController hwC, Telemetry t) {
		super(gp);
		telemetry = t;
		motor = hwC;
		PIDController = new PID(armP, armI, armD, motor.getPos());
	}

	@Override
	protected void autonomousUpdate() {
		PIDController.setSetPoint(position);
		double power = PIDController.PIDLoop((double)motor.getPos());
		motor.setSpeed(power);
	}

	@Override
	protected void driverUpdate() {
		if (gamepad.left_bumper) {
			if (!isLBDown) {
				//gets run only once when first pressed
				wobbleArmPositionSetpoints++;
				if (wobbleArmPositionSetpoints > 2) {
					wobbleArmPositionSetpoints = 1;
				}
			}
			isLBDown = true;
		} else {
			isLBDown = false;
		}

		switch (wobbleArmPositionSetpoints) {
			case 1:
				position = 1.85;
				break;
			case 2:
				position = 0.11;
				break;
			default:

		}

		PIDController.setSetPoint(position);
		double power = PIDController.PIDLoop((double)motor.getVoltage());
		power = -power;
		if (gamepad.a) {
			power = -0.5;
		}
		telemetry.addData("Arm Power: ", power);
		telemetry.addData("Arm SetPoint: ", position);
		telemetry.addData("Arm Position: ", motor.getVoltage());

		motor.setSpeed(power);
	}

	public boolean setArmPosition(double pos) {
		position = pos;
		return Math.abs(motor.getPos() - pos) < 0.1;
	}

	public double getArmPosition() {
		return motor.getVoltage();
	}
}