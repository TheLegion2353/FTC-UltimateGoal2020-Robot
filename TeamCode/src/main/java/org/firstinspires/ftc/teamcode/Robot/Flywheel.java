package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.*;

public class Flywheel extends RobotPart {
	private HardwareControllerEx flywheel = null;
	private PID pid = null;
	private double lastPosition = 0;
	private double lastTime = 0;
	private double lastVelocity = 0;
	private Telemetry telemetry = null;
	private double averageVoltage = 0;

	public Flywheel(Gamepad gp, Telemetry t, DcMotorEx... motors) {
		super(gp);
		telemetry = t;
		pid = new PID(0.1, 0.0, 0.0, 0);
		flywheel = new HardwareControllerEx(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, motors);
		flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}

	@Override
	public void driverUpdate() {
		double velocity = flywheel.getVelocity() / 2 / Math.PI * 10;  // convert radians/second (getVelocity()) to rev/second.
		telemetry.addData("Flywheel position:", flywheel.getPos());
		double power = pid.PIDLoop(velocity);
		telemetry.addData("Average Power: ", power);
		telemetry.addData("Average Velocity", velocity);

		if (power < 0){
			power = 0;
		}

		if (gamepad.y) {
			pid.setSetPoint(35);
			flywheel.setSpeed(power);
		} else if (gamepad.b) {
			pid.setSetPoint(30);
			flywheel.setSpeed(power);
		} else {
			pid.setSetPoint(0);
			flywheel.setSpeed(0);
		}
	}

	@Override
	public void autonomousUpdate() {
		double velocity = flywheel.getVelocity();
		double power = pid.PIDLoop(velocity);
		averageVoltage += power;
		averageVoltage /= 2;
		telemetry.addData("Average Power: ", averageVoltage);
		telemetry.addData("Velocity", velocity);
		if (averageVoltage < 0){
			averageVoltage = 0;
		}
		flywheel.setSpeed(averageVoltage);
	}

	public boolean setSpeed(double s) {
		pid.setSetPoint(s);
		return((Math.abs(flywheel.getVelocity() - s) < 5) || s == 0);
	}

	public void updatePID() {
	}
}