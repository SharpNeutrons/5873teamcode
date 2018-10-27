package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Landervator {

	protected DcMotor extensionMotor;
	protected DcMotor pitch;
	protected Servo extensionLock;

	private OpMode5873 opMode;
	private MultipleTelemetry telemetry;

	//TODO get real values for all of these
	protected final int MAX_EXTENDED_COUNTS = 10000;
	protected final int MAX_PITCH_COUNTS = 10000;
	protected final double EXT_LOCK_OPENED = 0.5;
	protected final double EXT_LOCK_CLOSED = 0.6;

	protected Landervator () {};

	protected void init (OpMode5873 _oM, HardwareMap hwm, MultipleTelemetry _t) {
		opMode = _oM;
		telemetry = _t;

		extensionMotor = hwm.get(DcMotor.class, "landerExtension");
		pitch = hwm.get(DcMotor.class, "landerPitch");

		extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		pitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		pitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		pitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		extensionLock = hwm.get(Servo.class, "extensionLock");
		extensionLock.setPosition(EXT_LOCK_CLOSED);
	}

	protected void teleOpControl (Gamepad gp1, Gamepad gp2) {
		if (extensionMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) &&
				extensionMotor.getCurrentPosition() < MAX_EXTENDED_COUNTS) {
			double extensionControl = Math.pow(-gp2.left_stick_y, 2);
			extensionMotor.setPower(extensionControl);
			if (gp2.dpad_up) {
				extendToPos(0/*TODO get full extension pos*/);
			}else if (gp2.dpad_down) {
				extendToPos(0);
			}
		}
		if (pitch.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) &&
				pitch.getCurrentPosition() < MAX_PITCH_COUNTS) {
			double pitchControl = Math.pow(-gp2.right_stick_y, 2) / 3;
			pitch.setPower(pitchControl);
			/*TODO when cratervator is made, look for something to move
				TODO it out of the way as this pitches up*/
			if (gp2.dpad_up) {
				pitchToPos(0/*TODO get full pitch pos*/);
			}else if (gp2.dpad_down) {
				pitchToPos(0);
			}
		}

		if (extensionMotor.getPower() == 0 && gp2.b) {
			extensionLock.setPosition(EXT_LOCK_CLOSED);
		}

		if (gp2.x) {
			extensionLock.setPosition(EXT_LOCK_OPENED);
		}

		telemetry.addData("Landervator Pos", extensionMotor.getCurrentPosition());
		telemetry.addData("Landerator pitch", pitch.getCurrentPosition());

		encoderLoop(extensionMotor);
		encoderLoop(pitch);

		testMotorLimit(extensionMotor, MAX_EXTENDED_COUNTS);
		testMotorLimit(pitch, MAX_PITCH_COUNTS);
	}

	private void extendToPos (int pos) {
		double speed = 0.5;
		extensionMotor.setTargetPosition(pos);
		extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		extensionMotor.setPower(speed);
	}

	private void pitchToPos (int pos) {
		double speed = 0.25;
		pitch.setTargetPosition(pos);
		pitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		pitch.setPower(speed);
	}

	private void encoderLoop (DcMotor motor) {
		if (!opMode.opModeIsActive() || !motor.isBusy()) {
			motor.setPower(0);
			motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
	}

	private void testMotorLimit (DcMotor motor, int limit) {
		if (motor.getCurrentPosition() > limit) {
			motor.setPower(0);
			motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
	}
}