package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.TaskScheduler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Main Autonomous", group="Autonomous")
public class MainAutonomousOp extends OpMode {
	public interface Runnable {
		public void run();
	}

	Robot robot = null;
	TaskScheduler taskScheduler = new TaskScheduler();
	SampleMecanumDrive drive = null;
	@Override
	public void init() {
		robot = new Robot(gamepad1, telemetry);
		drive = new SampleMecanumDrive(hardwareMap);
		taskScheduler.addTasks();
	}

	@Override
	public void loop() {
		drive.update();
		Pose2d poseEstimate = drive.getPoseEstimate();
		telemetry.addData("x", poseEstimate.getX());
		telemetry.addData("y", poseEstimate.getY());
		telemetry.addData("heading", poseEstimate.getHeading());
		telemetry.update();
	}

	@Override
	public void stop() {
		PoseStorage.currentPos = drive.getPoseEstimate();
	}
/*
	public class move implements Runnable{
		public void run {
			System.out.println("hello");
		}
	}

 */
}