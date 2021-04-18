package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.*;

public class Flywheel extends RobotPart {
	private HardwareController flywheel = null;
	private PID pid = null;
	private double lastPosition = 0;
	private double lastTime = 0;
	private double lastVelocity = 0;
	private Telemetry telemetry = null;
	private double averageVelocity = 0;
	private double averageVoltage = 0;

	public Flywheel(Gamepad gp, Telemetry t, DcMotor ... motors) {
		super(gp);
		telemetry = t;
		pid = new PID(0.075, 0.04, 0.0, 0);
		flywheel = new HardwareController(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
		flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}

	@Override
	public void driverUpdate() {
		double velocity = (double)(flywheel.getPos() - lastPosition) / ((double)(System.currentTimeMillis() - lastTime) / 1000);
		velocity /= 28;
		lastPosition = flywheel.getPos();
		lastTime = System.currentTimeMillis();
		averageVelocity = velocity;
		double power = pid.PIDLoop(averageVelocity);
		averageVoltage += power;
		averageVoltage /= 2;
		telemetry.addData("Average Power: ", averageVoltage);
		telemetry.addData("Average Velocity", averageVelocity);
		if (averageVoltage < 0){
			averageVoltage = 0;
		}
		if (gamepad.y) {
			pid.setSetPoint(65);
			flywheel.setSpeed(averageVoltage);
		} else if (gamepad.b) {
			pid.setSetPoint(60);
			flywheel.setSpeed(averageVoltage);
		} else if (gamepad.x) {
			flywheel.setSpeed(0.7);
		} else {
			pid.setSetPoint(0);
			flywheel.setSpeed(0);
		}
	}

	@Override
	public void autonomousUpdate() {
		double velocity = (double)(flywheel.getPos() - lastPosition) / ((double)(System.currentTimeMillis() - lastTime) / 1000);
		velocity /= 28;
		lastPosition = flywheel.getPos();
		lastTime = System.currentTimeMillis();
		averageVelocity = velocity;
		double power = pid.PIDLoop(averageVelocity);
		averageVoltage += power;
		averageVoltage /= 2;
		telemetry.addData("Average Power: ", averageVoltage);
		telemetry.addData("Average Velocity", averageVelocity);
		if (averageVoltage < 0){
			averageVoltage = 0;
		}
		flywheel.setSpeed(averageVoltage);
	}

	public boolean setSpeed(double s) {
		pid.setSetPoint(s);
		return((Math.abs(averageVelocity - s) < 5) || s == 0);
	}

	public void updatePID() {
	}
}