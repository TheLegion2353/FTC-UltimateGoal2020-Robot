package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


public class Drivetrain extends RobotPart {
	private ElapsedTime clock = null;
	private ControlType control;
	private HardwareController topRight = null;
	private HardwareController topLeft = null;
	private HardwareController bottomRight = null;
	private HardwareController bottomLeft = null;
	private Telemetry telemetry = null;

	public Drivetrain(ControlType ct, Gamepad gp, Telemetry t) {
		super(gp);
		telemetry = t;
		control = ct;
		topLeft = new HardwareController();
		topRight = new HardwareController();
		bottomLeft = new HardwareController();
		bottomRight = new HardwareController();
	}

	public Drivetrain(ControlType ct, Gamepad gp) {
		super(gp);
		control = ct;
		topLeft = new HardwareController();
		topRight = new HardwareController();
		bottomLeft = new HardwareController();
		bottomRight = new HardwareController();
	}

	@Override
	protected void driverUpdate() {
		if (gamepad != null) {
			double leftPower = gamepad.left_stick_y - gamepad.right_stick_x;
			double rightPower = -gamepad.left_stick_y - gamepad.right_stick_x;
			double strafePower = gamepad.left_stick_x;
			topLeft.setSpeed(leftPower - strafePower);
			topRight.setSpeed(rightPower - strafePower);
			bottomLeft.setSpeed(leftPower + strafePower);
			bottomRight.setSpeed(rightPower + strafePower);
		}
	}

	public void setTopLeft(DcMotor.RunMode mode, DcMotor ... motors) {
		topLeft = new HardwareController(mode, motors);
	}

	public void setTopRight(DcMotor.RunMode mode, DcMotor ... motors) {
		topRight = new HardwareController(mode, motors);
	}

	public void setBottomLeft(DcMotor.RunMode mode, DcMotor ... motors) {
		bottomLeft = new HardwareController(mode, motors);
	}

	public void setBottomRight(DcMotor.RunMode mode, DcMotor ... motors) {
		bottomRight = new HardwareController(mode, motors);
	}

	public enum ControlType {
		TANK,
		ARCADE
	}
}