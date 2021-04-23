package org.firstinspires.ftc.teamcode.Robot;
import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HardwareControllerEx {
	private ArrayList<DcMotorEx> motors = null;
	private ArrayList<Servo> servos = null;
	private ArrayList<CRServo> crservos = null;
	private DcMotorEx.Direction direction = DcMotorEx.Direction.FORWARD;
	private AnalogInput potentiometer = null;

	public HardwareControllerEx() {
		motors = new ArrayList<DcMotorEx>();
		servos = new ArrayList<Servo>();
		crservos = new ArrayList<CRServo>();
	}

	public HardwareControllerEx(DcMotorEx.RunMode mode, DcMotorEx ... motorArgs) {
		motors = new ArrayList<DcMotorEx>();
		servos = new ArrayList<Servo>();
		crservos = new ArrayList<CRServo>();
		for (DcMotorEx mot : motorArgs) {
			mot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
			addMotor(mot, mode);
		}
	}

	public void addMotor(DcMotorEx motor, DcMotorEx.RunMode mode) {
		motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(mode);
		motor.setDirection(direction);
		motors.add(motor);
	}

	public void addAnalogInput(AnalogInput device) {
		potentiometer = device;
	}

	public void setSpeed(double s) {
		for (DcMotorEx m : motors) {
			m.setPower(s);
		}

		for (CRServo cr : crservos) {
			cr.setPower(s);
		}
	}

	public void setPosition(double p) {
		for (Servo s : servos) {
			s.setPosition(p);
		}
	}

	public void resetEncoders(DcMotorEx.RunMode mode) {
		for (DcMotorEx m : motors) {
			m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
			m.setMode(mode);
		}
	}

	public double getPos() {
		double avrgPos = 0;
		for (DcMotorEx m : motors) {
			avrgPos += m.getCurrentPosition();
		}
		avrgPos /= (double)motors.size();
		return avrgPos;
	}

	public double getVoltage() {
		return potentiometer.getVoltage();
	}
	public void addMotor(DcMotorEx motor) {
		motors.add(motor);
	}

	public void addServo(Servo servo) {
		servos.add(servo);
	}

	public void addCRServo(CRServo crservo) {
		crservos.add(crservo);
	}

	public void setDirection(DcMotorEx.Direction dir) {
		for (DcMotorEx m : motors) {
			m.setDirection(dir);
		}
	}

	public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior b) {
		for (DcMotorEx m : motors) {
			m.setZeroPowerBehavior(b);
		}
	}

	public double getVelocity() {
		double avrgVel = 0;
		for (DcMotorEx m : motors) {
			avrgVel += m.getVelocity(AngleUnit.RADIANS);
		}
		avrgVel /= (double)motors.size();
		return avrgVel;
	}
}