package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Robot {
	private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	private Drivetrain mecanum;
	private Gamepad gamepad = null;
	private Telemetry telemetry = null;
	private boolean isWaiting = false;
	private double endTime = 0;
	double xPosition = 0; //inches
	double yPosition = 0;
	double angle = 0;

	public Robot(Gamepad gp, Telemetry t) {
		telemetry = t;
		gamepad = gp;
		mecanum = new Drivetrain(Drivetrain.ControlType.ARCADE, gamepad, t);
	}

	public Robot(Gamepad gp) {
		gamepad = gp;
	}

	public void update() {
		double time = (double)clock.time(TimeUnit.MILLISECONDS) / 1000.0;
		mecanum.update();
		clock.reset();
	}

	public void setTopLeft(DcMotor ... motors) {
		mecanum.setTopLeft(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
	}

	public void setTopRight(DcMotor ... motors) {
		mecanum.setTopRight(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
	}

	public void setBottomLeft(DcMotor ... motors) {
		mecanum.setBottomLeft(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
	}

	public void setBottomRight(DcMotor ... motors) {
		mecanum.setBottomRight(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
	}
}