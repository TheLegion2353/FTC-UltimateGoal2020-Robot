package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.Claw;
import org.firstinspires.ftc.teamcode.Robot.Extensions;
import org.firstinspires.ftc.teamcode.Robot.Flywheel;
import org.firstinspires.ftc.teamcode.Robot.HardwareController;
import org.firstinspires.ftc.teamcode.Robot.Whacker;
import org.firstinspires.ftc.teamcode.drive.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Main Mecanum", group="Driver Controlled")
public class MainTeleOp extends OpMode {
	private SampleMecanumDrive drive = null;
	private Flywheel flywheel = null;
	private Arm arm = null;
	private HardwareController intake = null;
	private Claw wobbleClaw = null;
	private Whacker whacker = null;
	private Extensions extensions = null;
	@Override
	public void init() {
		drive = new SampleMecanumDrive(hardwareMap);
		drive.setPoseEstimate(PoseStorage.currentPos);  //change to teleop start position
		flywheel = new Flywheel(gamepad1, telemetry, hardwareMap.get(DcMotorEx.class, "flywheel"));
		HardwareController hwC = new HardwareController(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap.get(DcMotor.class, "wobble/lateral"));
		hwC.addAnalogInput(hardwareMap.get(AnalogInput.class, "wobblePotentiometer"));
		arm = new Arm(gamepad1, hwC, telemetry);
		intake = new HardwareController(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, hardwareMap.get(DcMotorEx.class, "intake/right"));
		whacker = new Whacker(gamepad1, hardwareMap.get(Servo.class, "whacker"));
		wobbleClaw = new Claw(gamepad1, hardwareMap.get(Servo.class, "claw"));
		extensions = new Extensions(gamepad1, hardwareMap.get(Servo.class, "extension1"), hardwareMap.get(Servo.class, "extension2"));
	}

	@Override
	public void loop() {
		drive.setWeightedDrivePower(
				new Pose2d(
						-gamepad1.left_stick_y,
						-gamepad1.left_stick_x,
						-gamepad1.right_stick_x
				)
		);

		drive.update();
		whacker.update();
		arm.update();
		flywheel.update();
		wobbleClaw.update();
		extensions.update();
		intake.setSpeed(gamepad1.right_trigger - gamepad1.left_trigger);

		Pose2d poseEstimate = drive.getPoseEstimate();
		telemetry.addData("x", poseEstimate.getX());
		telemetry.addData("y", poseEstimate.getY());
		telemetry.addData("heading", poseEstimate.getHeading());
		telemetry.update();
	}
}