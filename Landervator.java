package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Landervator {

	protected DcMotor extensionMotor;
	protected DcMotor pitchMotor;
	protected Servo extensionLock;

	private boolean extensionLocked = true;

	private OpMode5873 opMode;
	private MultipleTelemetry telemetry;

	protected enum LOCK_POS {OPENED, LOCKED}

	//TODO get real values for all of these
	protected final int MAX_EXTENDED_COUNTS = 10000;
	protected final int MAX_PITCH_COUNTS = 10000;
	protected final double EXT_LOCK_OPENED = 0.5;
	protected final double EXT_LOCK_CLOSED = 0.6;
	protected final int MIN_LATCH_COUNTS = 5000;
	protected final int MIN_UNLATCH_COUNTS = 8000;

	protected Landervator () {};

	protected void init (OpMode5873 _oM, HardwareMap hwm, MultipleTelemetry _t) {
		opMode = _oM;
		telemetry = _t;

		extensionMotor = hwm.get(DcMotor.class, "landerExtension");
		pitchMotor = hwm.get(DcMotor.class, "landerPitch");

		extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		pitchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		pitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		extensionLock = hwm.get(Servo.class, "extensionLock");
		extensionLock.setPosition(EXT_LOCK_CLOSED);
	}

	protected void setExtensionSpeed (double speed) {
		if (extensionMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) &&
				extensionMotor.getCurrentPosition() < MAX_EXTENDED_COUNTS) {
			extensionMotor.setPower(speed);
		}
		testMotorLimit(extensionMotor, MAX_EXTENDED_COUNTS);
		telemetry.addData("Landervator Pos", extensionMotor.getCurrentPosition());
	}

	protected void setPitchSpeed (double speed) {
		if (pitchMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) &&
				pitchMotor.getCurrentPosition() < MAX_PITCH_COUNTS) {
			pitchMotor.setPower(speed);
			/*TODO when cratervator is made, look for something to move
				TODO it out of the way as this pitches up*/
		}
		testMotorLimit(pitchMotor, MAX_PITCH_COUNTS);
		telemetry.addData("Landervator pitchMotor", pitchMotor.getCurrentPosition());
	}

	protected void runEncoderLoop () {
		encoderLoop(extensionMotor);
		encoderLoop(pitchMotor);
	}

	protected void extendToPos (int pos) {
		double speed = 0.5;
		extensionMotor.setTargetPosition(pos);
		extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		extensionMotor.setPower(speed);
	}

	protected void pitchToPos (int pos) {
		double speed = 0.25;
		pitchMotor.setTargetPosition(pos);
		pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		pitchMotor.setPower(speed);
	}

	protected void moveLock (LOCK_POS pos) {
		if (pos.equals(LOCK_POS.LOCKED)) {
			if (extensionMotor.getPower() == 0 && !extensionMotor.isBusy()) {
				extensionLock.setPosition(EXT_LOCK_CLOSED);
				extensionLocked = true;
			}
		}else if (pos.equals(LOCK_POS.OPENED)) {
			if (extensionMotor.getPower() == 0 && !extensionMotor.isBusy()) {
				extensionLock.setPosition(EXT_LOCK_OPENED);
				extensionLocked = false;
			}
		}
	}

	private void encoderLoop (DcMotor motor) {
		if (!opMode.opModeIsActive() || !motor.isBusy()) {
			motor.setPower(0);
			motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
	}

	private void testMotorLimit (DcMotor motor, int limit) {
		if (motor.getCurrentPosition() > limit || extensionLocked) {
			motor.setPower(0);
			motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
	}
}