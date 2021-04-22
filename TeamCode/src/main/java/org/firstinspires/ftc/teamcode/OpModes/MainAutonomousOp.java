package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.Claw;
import org.firstinspires.ftc.teamcode.Robot.HardwareController;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Whacker;
import org.firstinspires.ftc.teamcode.TaskScheduler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="Main Autonomous", group="Autonomous")
public class MainAutonomousOp extends LinearOpMode {

	private SampleMecanumDrive drive = null;
	private Claw wobbleClaw = null;
	private HardwareController hwC = null;
	private Arm arm = null;
	private Whacker whacker = null;
	private DcMotor fly = null;
	private AutoPath path = AutoPath.A;
	//tensorflow
	private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
	private static final String LABEL_FIRST_ELEMENT = "Quad";
	private static final String LABEL_SECOND_ELEMENT = "Single";
	private TFObjectDetector tfod;
	private double forwardFactor = 0.5;

	// Vuforia Related Things:
	VuforiaTrackables targetsUltimateGoal;
	private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
	private static final boolean PHONE_IS_PORTRAIT = false  ;
	private static final String VUFORIA_KEY =
			"Aecx07L/////AAABmQbDimTWOUPdkStEP3xpsklJgTSeNK1GUg1sse6qFp4arinGemTI6WwY5YGIKzR5yXW7hzwB+4aLFEDVBIz7EsMvWbH3LG4FeTLS7HiSFFrC1gtOx31tlNZTxNtr9hoOBKi0NAgaQKlMLGGz2xj4Dnw8uxUKZPh7/V9s5NRI2n1LZIGczBGMWJB3UO0wmjk3wKsuFxl519fpP7C53g1z9d9f74KyAGhDNrEXUELNIYHgaAPjDj2BMHpwFxJh0om1l90Hx3/pq6dbh6LFAPHpLVtbBkB9zPLfdIPqTwWEuim00LDY4gT4eHZfvIMD8G5BoigjC8KS9/kIJ5TklVUE4orSsl15oPqJv1t3tDRYZRDu";
	// Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
	// We will define some constants and conversions here
	private static final float mmPerInch        = 25.4f;
	private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

	// Constants for perimeter targets
	private static final float halfField = 72 * mmPerInch;
	private static final float quadField  = 36 * mmPerInch;

	// Class Members
	private OpenGLMatrix lastLocation = null;
	private VuforiaLocalizer vuforia = null;

	WebcamName webcamName = null;

	private boolean targetVisible = false;
	private float phoneXRotate    = 0;
	private float phoneYRotate    = 0;
	private float phoneZRotate    = 0;

	List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
	@Override
	public void runOpMode() throws InterruptedException {
		//TensorFlow
		webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
		 * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
		 */
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

		// VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		/**
		 * We also indicate which camera on the RC we wish to use.
		 */
		parameters.cameraName = webcamName;

		// Make sure extended tracking is disabled for this example.
		parameters.useExtendedTracking = false;

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Load the data sets for the trackable objects. These particular data
		// sets are stored in the 'assets' part of our application.
		targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
		VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
		blueTowerGoalTarget.setName("Blue Tower Goal Target");
		VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
		redTowerGoalTarget.setName("Red Tower Goal Target");
		VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
		redAllianceTarget.setName("Red Alliance Target");
		VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
		blueAllianceTarget.setName("Blue Alliance Target");
		VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
		frontWallTarget.setName("Front Wall Target");

		// For convenience, gather together all the trackable objects in one easily-iterable collection */
		allTrackables.addAll(targetsUltimateGoal);

		/**
		 * In order for localization to work, we need to tell the system where each target is on the field, and
		 * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
		 * Transformation matrices are a central, important concept in the math here involved in localization.
		 * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
		 * for detailed information. Commonly, you'll encounter transformation matrices as instances
		 * of the {@link OpenGLMatrix} class.
		 *
		 * If you are standing in the Red Alliance Station looking towards the center of the field,
		 *     - The X axis runs from your left to the right. (positive from the center to the right)
		 *     - The Y axis runs from the Red Alliance Station towards the other side of the field
		 *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
		 *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
		 *
		 * Before being transformed, each target image is conceptually located at the origin of the field's
		 *  coordinate system (the center of the field), facing up.
		 */

		//Set the position of the perimeter targets with relation to origin (center of field)
		redAllianceTarget.setLocation(OpenGLMatrix
				.translation(0, -halfField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

		blueAllianceTarget.setLocation(OpenGLMatrix
				.translation(0, halfField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
		frontWallTarget.setLocation(OpenGLMatrix
				.translation(-halfField, 0, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

		// The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
		blueTowerGoalTarget.setLocation(OpenGLMatrix
				.translation(halfField, quadField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
		redTowerGoalTarget.setLocation(OpenGLMatrix
				.translation(halfField, -quadField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

		//
		// Create a transformation matrix describing where the phone is on the robot.
		//
		// NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
		// Lock it into Portrait for these numbers to work.
		//
		// Info:  The coordinate frame for the robot looks the same as the field.
		// The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
		// Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
		//
		// The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
		// pointing to the LEFT side of the Robot.
		// The two examples below assume that the camera is facing forward out the front of the robot.

		// We need to rotate the camera around it's long axis to bring the correct camera forward.
		if (CAMERA_CHOICE == BACK) {
			phoneYRotate = -90;
		} else {
			phoneYRotate = 90;
		}

		// Rotate the phone vertical about the X axis if it's in portrait mode
		if (PHONE_IS_PORTRAIT) {
			phoneXRotate = 90 ;
		}

		// Next, translate the camera lens to where it is on the robot.
		// In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
		final float CAMERA_FORWARD_DISPLACEMENT  = 7.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
		final float CAMERA_VERTICAL_DISPLACEMENT = 12.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
		final float CAMERA_LEFT_DISPLACEMENT     = 5.0f	* mmPerInch;     // eg: Camera is ON the robot's center line

		OpenGLMatrix robotFromCamera = OpenGLMatrix
				.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

		/**  Let all the trackable listeners know where the phone is.  */
		for (VuforiaTrackable trackable : allTrackables) {
			((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
		}

		// WARNING:
		// In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
		// This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
		// CONSEQUENTLY do not put any driving commands in this loop.
		// To restore the normal opmode structure, just un-comment the following line:

		// waitForStart();

		// Note: To use the remote camera preview:
		// AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
		// Tap the preview window to receive a fresh image.

		targetsUltimateGoal.activate();
		initTfod();
		if (tfod != null) {
			tfod.activate();

			// The TensorFlow software will scale the input images from the camera to a lower resolution.
			// This can result in lower detection accuracy at longer distances (> 55cm or 22").
			// If your target is at distance greater than 50 cm (20") you can adjust the magnification value
			// to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
			// should be set to the value of the images used to create the TensorFlow Object Detection model
			// (typically 1.78 or 16/9).

			// Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
			//tfod.setZoom(2.5, 1.78);
		}


		//init
		wobbleClaw = new Claw(gamepad1, hardwareMap.get(Servo.class, "claw"));
		hwC = new HardwareController(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap.get(DcMotor.class, "wobble/lateral"));
		hwC.addAnalogInput(hardwareMap.get(AnalogInput.class, "wobblePotentiometer"));
		arm = new Arm(gamepad1, hwC, telemetry);
		wobbleClaw.setGrab(1);
		whacker = new Whacker(gamepad1, hardwareMap.get(Servo.class, "whacker"));
		whacker.setPosition(0);
		fly = hardwareMap.get(DcMotor.class, "flywheel");
		fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		drive = new SampleMecanumDrive(hardwareMap);
		Pose2d startPose = new Pose2d(-60, -60, Math.toRadians(0));
		drive.setPoseEstimate(startPose);
		Trajectory common = drive.trajectoryBuilder(startPose)
				.lineToLinearHeading(new Pose2d(0,-45, Math.toRadians(90)))
				.build();

		while(!opModeIsActive()) {
			if (isStopRequested()) return;
			TFLoop();
		}

		Trajectory dropZone = null;
		if (path == AutoPath.A) {  // A
			telemetry.addLine("Path A");
			dropZone = drive.trajectoryBuilder(common.end())
					.lineToConstantHeading(new Vector2d(10,-45))
					.build();
		} else if (path == AutoPath.B) {  // B
			telemetry.addLine("Path B");

			dropZone = drive.trajectoryBuilder(common.end())
					.lineToConstantHeading(new Vector2d(28,-17))
					.build();
		} else {  // C
			telemetry.addLine("Path C");

			dropZone = drive.trajectoryBuilder(common.end())
					.lineToConstantHeading(new Vector2d(50,-40))
					.build();
		}
		telemetry.update();

		Trajectory highGoal = drive.trajectoryBuilder(dropZone.end())
				.lineToLinearHeading(new Pose2d(-5,-25, Math.toRadians(0)))
				.build();

		Trajectory lineUpForWobble = null;
		if (path == AutoPath.A) {
			lineUpForWobble  = drive.trajectoryBuilder(highGoal.end())
					.lineToConstantHeading(new Vector2d(-35, -17.5))
					.build();
		} else if (path == AutoPath.B) {
			lineUpForWobble  = drive.trajectoryBuilder(highGoal.end())
					.lineToConstantHeading(new Vector2d(-40, -22))
					.build();
		} else {
			lineUpForWobble  = drive.trajectoryBuilder(highGoal.end())
					.lineToConstantHeading(new Vector2d(-39, -21))
					.build();
		}

		Trajectory wobble2;
		if (path == AutoPath.A) {
			wobble2 = drive.trajectoryBuilder(lineUpForWobble.end())
					.lineToConstantHeading(new Vector2d(-36.5, -18.25))
					.build();
		} else if (path == AutoPath.B) {
			wobble2 = drive.trajectoryBuilder(lineUpForWobble.end())
					.lineToConstantHeading(new Vector2d(-41, -21))
					.build();
		} else {
			wobble2 = drive.trajectoryBuilder(lineUpForWobble.end())
					.lineToConstantHeading(new Vector2d(-40, -22))
					.build();
		}
		Trajectory common2 = drive.trajectoryBuilder(wobble2.end())
				.lineToLinearHeading(new Pose2d(-18, 0, Math.toRadians(90)))
				.build();

		Trajectory dropZone2 = null;
		if (path == AutoPath.A) {  // A
			dropZone2 = drive.trajectoryBuilder(common2.end())
					.lineToConstantHeading(new Vector2d(-5,-45))
					.build();
		} else if (path == AutoPath.B) {  // B
			dropZone2 = drive.trajectoryBuilder(common2.end())
					.lineToConstantHeading(new Vector2d(17.5,-15))
					.build();
		} else {  // C
			dropZone2 = drive.trajectoryBuilder(common2.end())
					.lineToConstantHeading(new Vector2d(32.5,-40))
					.build();
		}

		Trajectory park = drive.trajectoryBuilder(dropZone2.end())
				.lineToConstantHeading(new Vector2d(0, -20))
				.build();


		if (isStopRequested()) return;

		drive.followTrajectory(common);
		drive.followTrajectory(dropZone);  // go to drop zone
		dropWobble();  // drop wobble at drop zone
		drive.followTrajectory(highGoal);  // go into shooting position
		shootRings();
		bringDownWobbleClaw();
		drive.followTrajectory(lineUpForWobble);
		drive.followTrajectory(wobble2);
		pickUpWobble();
		drive.followTrajectory(common2);
		drive.followTrajectory(dropZone2);
		dropWobble();
		drive.followTrajectory(park);
		//ending
		PoseStorage.currentPos = drive.getPoseEstimate();  // store pose for teleop
	}

	private void dropWobble() throws InterruptedException {
		arm.update();
		while (!arm.setArmPosition(0.11)) {
			arm.update();
		}
		wobbleClaw.setGrab(0);
		Thread.sleep(200);
		while (!arm.setArmPosition(1.85)) {
			arm.update();
		}
	}

	private void bringDownWobbleClaw() {
		arm.update();
		while (!arm.setArmPosition(0.11)) {
			arm.update();
		}
	}
	private void pickUpWobble() throws InterruptedException {
		arm.update();
		wobbleClaw.setGrab(1);
		Thread.sleep(600);
		while (!arm.setArmPosition(1.85)) {
			arm.update();
		}
	}

	private void shootRings() throws InterruptedException {
		spinUpFlywheel();
		int amnt = 5;
		if (path == AutoPath.C) {
			amnt = 3;
		}
		for (int i = 0; i < amnt; i++) {
			whacker.setPosition(1);
			Thread.sleep(550);
			whacker.setPosition(0);
			Thread.sleep(250);
		}
		fly.setPower(0);
	}

	private void spinUpFlywheel() throws InterruptedException {
		fly.setPower(0.5);  // on normal voltages, .75
		Thread.sleep(1500);
	}

	private void TFLoop() {
		//TensorFlow detection
		if (tfod != null) {
			// getUpdatedRecognitions() will return null if no new information is available since
			// the last time that call was made.
			List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
			if (updatedRecognitions != null) {
				telemetry.addData("# Object Detected", updatedRecognitions.size());
				// step through the list of recognitions and display boundary info.
				int i = 0;
				path = AutoPath.A;
				for (Recognition recognition : updatedRecognitions) {
					telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
					if (recognition.getLabel() == "Quad") {
						path = AutoPath.C;
					} else if (recognition.getLabel() == "Single") {
						path = AutoPath.B;
					} else {
						path = AutoPath.A;
					}
					telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
							recognition.getLeft(), recognition.getTop());
					telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
							recognition.getRight(), recognition.getBottom());
				}
				telemetry.update();
			}
		} else {
			path = AutoPath.A;
		}
	}

	private void initTfod() { //initialize TensorFlow
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minResultConfidence = 0.5f;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
	}

	private enum AutoPath {
		A,
		B,
		C
	}
}