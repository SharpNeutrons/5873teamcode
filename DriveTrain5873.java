package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTrain5873 {

	protected DcMotor leftDrive;
	protected DcMotor rightDrive;

	private LinearOpMode opMode;
	private MultipleTelemetry telemetry;

	protected final double WHEEL_RADUIS_MM = 50.8;
	protected final double COUNTS_PER_MM_DRIVE = /*HardwareDTBot.HD_40_COUNTS_PER_REV*/1120 / (WHEEL_RADUIS_MM * 2 * Math.PI);
	protected final double WHEEL_DISTANCE_FROM_CENTER_MM = 239.7;//237.015;//just a rough thing from the cad for now

	//protected final double STEER_WEIGHT = 0.75;
	//protected final double DRIVE_WEIGHT = 1 - STEER_WEIGHT;
	private ElapsedTime runtime = new ElapsedTime();

	protected DriveTrain5873 () {}

	protected void init (LinearOpMode _oM, HardwareMap hwm, MultipleTelemetry _t) {
		opMode = _oM;
		telemetry = _t;

		leftDrive = hwm.get(DcMotor.class, "leftDrive");
		rightDrive = hwm.get(DcMotor.class, "rightDrive");

		leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
		rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

		leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	protected void setLeftRightPower (double power) {
		leftDrive.setPower(power);
		rightDrive.setPower(power);
	}

	protected void setLeftRightMode (DcMotor.RunMode mode) {
		leftDrive.setMode(mode);
		rightDrive.setMode(mode);
	}

	/**
	 *
	 * @param distanceMM how far the robot should drive in millimeters, can be + or -
	 * @param speed 0 to 1 as a fraction of motor power
	 * @param timeout a time in ms that the drive should stop after if all else fails
	 */
	protected void driveEncoderDistance (double distanceMM, double speed, double timeout) {
		double currentCountLeft = leftDrive.getCurrentPosition();
		double currentCountRight = rightDrive.getCurrentPosition();

		double countsToDrive = COUNTS_PER_MM_DRIVE * distanceMM;

		int leftTarget = (int)(currentCountLeft + countsToDrive);
		int rightTarget = (int)(currentCountRight + countsToDrive);

		telemetry.addLine("targets defined");
		telemetry.update();
		moveMotorEncodersToTargets(leftTarget, rightTarget, Math.abs(speed), timeout);
	}

	/**
	 * @param degrees Degrees to spin, + ccw, - cw
	 * @param speed speed from 0 to 1 at which to rotate
	 * @param timeout the max time the rotation should take
	 */
	protected void rotateByEncoders (double degrees, double speed, double timeout) {
		double currentLeft = leftDrive.getCurrentPosition();
		double currentRight = rightDrive.getCurrentPosition();

		//TODO figure out the factor of ~3/4 that is somewhere in the code
		double radians = degrees * Math.PI / 180;
		double countsToDrive = WHEEL_DISTANCE_FROM_CENTER_MM * radians * COUNTS_PER_MM_DRIVE;

		int leftTarget = (int)(currentLeft - countsToDrive);
		int rightTarget = (int)(currentRight + countsToDrive);

		moveMotorEncodersToTargets(leftTarget, rightTarget, Math.abs(speed), timeout);
	}

	/**
	 *
	 * @param leftTarget leftDrive's encoder target
	 * @param rightTarget rightDrive's encoder target
	 * @param speed the speed at which the two motors should move
	 * @param timeout how long the motors should be given to move, in ms
	 */
	private void moveMotorEncodersToTargets (int leftTarget, int rightTarget, double speed, double timeout) {
		leftDrive.setTargetPosition(leftTarget);
		rightDrive.setTargetPosition(rightTarget);

		telemetry.addLine("targets set");
		telemetry.update();

		setLeftRightMode(DcMotor.RunMode.RUN_TO_POSITION);
		setLeftRightPower(speed);

		telemetry.addLine("modes and powers set");

//		telemetry.addData("left", leftDrive.getMode());
//		telemetry.addData("right", rightDrive.getMode());
//		telemetry.update();
//		opMode.sleep(2000);

		runtime.reset();
		//TODO start the loop to wait until the motor have reached their target positions
		while(opMode.opModeIsActive() && runtime.milliseconds() < timeout &&
				(leftDrive.isBusy() || rightDrive.isBusy())) {
			encoderLoop(leftTarget, rightTarget);
		}

		setLeftRightPower(0);
		setLeftRightMode(DcMotor.RunMode.RUN_USING_ENCODER);
		telemetry.addLine("movement complete");

	}

	private void encoderLoop (double leftTarget, double rightTarget) {
		telemetry.addData("left", leftDrive.getPower());
		telemetry.addData("right", rightDrive.getPower());
		telemetry.addData("left","%7d/%7d", leftDrive.getCurrentPosition(), leftTarget);
		telemetry.addData("right", "%7d/%7d", rightDrive.getCurrentPosition(), rightTarget);
		telemetry.update();
		opMode.idle();
	}
}
