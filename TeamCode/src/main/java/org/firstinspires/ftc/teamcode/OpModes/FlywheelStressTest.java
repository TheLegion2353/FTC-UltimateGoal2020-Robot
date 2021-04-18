package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.Flywheel;
import org.firstinspires.ftc.teamcode.Robot.HardwareController;
import org.firstinspires.ftc.teamcode.drive.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Flywheel Stress Test", group="Driver Controlled")
public class FlywheelStressTest extends OpMode {
	private SampleMecanumDrive drive = null;
	private Flywheel flywheel = null;
	private Arm arm = null;
	private HardwareController intake = null;

	@Override
	public void init() {
		drive = new SampleMecanumDrive(hardwareMap);
		drive.setPoseEstimate(PoseStorage.currentPos);  //change to teleop start position
		flywheel = new Flywheel(null, telemetry, hardwareMap.get(DcMotorEx.class, "flywheel"));
		HardwareController hwC = new HardwareController(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap.get(DcMotorEx.class, "wobble/lateral"));
		hwC.addAnalogInput(hardwareMap.get(AnalogInput.class, "wobblePotentiometer"));
		arm = new Arm(gamepad1, hwC, telemetry);
		intake = new HardwareController(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap.get(DcMotorEx.class, "intake/right"));
		flywheel.setSpeed(1000);
	}

	@Override
	public void loop() {
		flywheel.update();
	}
}