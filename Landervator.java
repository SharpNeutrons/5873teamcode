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

	protected boolean encoderMovementActive = false;

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

	/**
	 * Tests if the motors should keep running to their target positions, and turns them off if they shouldn't
	 */
	protected boolean runEncoderLoop () {
		encoderMovementActive = encoderLoop(extensionMotor) && encoderLoop(pitchMotor);
		return encoderMovementActive;
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

	private boolean encoderLoop (DcMotor motor) {
		if (!opMode.opModeIsActive() || !motor.isBusy()) {
			motor.setPower(0);
			motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			return false;
		}else {
			return true;
		}
	}

	protected void testMotorLimit (DcMotor motor, int limit) {
		if (motor.getCurrentPosition() > limit || extensionLocked) {
			motor.setPower(0);
			motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
	}
}