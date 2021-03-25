package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Mecanum DriveTrain", group="Driver Controlled")
public class MainTeleOp extends OpMode {
	Robot robot = null;

	@Override
	public void init() {
		robot = new Robot(gamepad1, telemetry);
		robot.setTopLeft(hardwareMap.get(DcMotor.class, "topLeft"));
		robot.setTopRight(hardwareMap.get(DcMotor.class, "topRight"));
		robot.setBottomLeft(hardwareMap.get(DcMotor.class, "bottomLeft"));
		robot.setBottomRight(hardwareMap.get(DcMotor.class, "bottomRight"));
	}

	@Override
	public void loop() {
		robot.update();
		telemetry.update();
	}
}