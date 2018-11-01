package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class AutoPilot {

	private DriveTrain5873 driveTrain;
	private Landervator landervator;
	//collector
	private SensoryManagement sensoryManagement;
	private MultipleTelemetry telemetry;
	private OpMode5873 opMode;

	private ElapsedTime runtime = new ElapsedTime();

	protected boolean driveTrainEncoderActive = false;//should the "loop" function in Hardware call
		//to check drive train movement

	protected enum DIRECTION {FORWARD, REVERSE}

	public AutoPilot (){}

	protected void init (OpMode5873 _op, DriveTrain5873 _dT, Landervator _l, SensoryManagement _sM, MultipleTelemetry _t){
		opMode = _op;
		driveTrain = _dT;
		landervator = _l;
		//collector
		sensoryManagement = _sM;
		telemetry = _t;
	}

	/**DRIVING STUFF**/
	/**Tells the robot to drive in a straight path to the next tape line (blue or red)
	 *
	 * @param speed 0 to 1 how fast the motors should go
	 * @param direction 1 for forward, -1 for backward
	 */
	protected void driveToLine (double speed, DIRECTION direction) {
		//TODO get working from pseudocode
		//double bkgSaturation = get saturation of current position from color sensors
		boolean lineDetected = false;

		speed = Range.clip(speed, 0, 1);
		if (direction.equals(DIRECTION.REVERSE)) {
			speed *= -1;
		}
		driveTrain.setLeftRightPower(speed);
		//in a while loop testing for opMode active
		while (opMode.opModeIsActive() && lineDetected) {
			//do a telemetry update
			//test if the saturation on the direction it is driving is above threshold
				//change line detected to true
		}
		driveTrain.setLeftRightPower(0);
	}

	//TODO move smart movement (like encoder stuff here) instead of in the respective classes
	/**
	 *
	 * @param distanceMM how far the robot should drive in millimeters, can be + or -
	 * @param speed 0 to 1 as a fraction of motor power
	 * @param timeout a time in ms that the drive should stop after if all else fails
	 */
	protected void driveEncoderDistance (double distanceMM, double speed, double timeout) {
		double currentCountLeft = driveTrain.leftDrive.getCurrentPosition();
		double currentCountRight = driveTrain.rightDrive.getCurrentPosition();

		double countsToDrive = driveTrain.COUNTS_PER_MM_DRIVE * distanceMM;

		int leftTarget = (int)(currentCountLeft + countsToDrive);
		int rightTarget = (int)(currentCountRight + countsToDrive);

		moveDriveMotorsByEncoder(leftTarget, rightTarget, speed, timeout);
	}

	/**
	 * @param degrees Degrees to spin, + ccw, - cw
	 * @param speed speed from 0 to 1 at which to rotate
	 * @param timeout the max time the rotation should take
	 */
	protected void rotateByEncoders (double degrees, double speed, double timeout) {
		double currentLeft = driveTrain.leftDrive.getCurrentPosition();
		double currentRight = driveTrain.rightDrive.getCurrentPosition();

		//TODO figure out the factor of ~3/4 that is somewhere in the code
		double radians = degrees * Math.PI / 180;
		double countsToDrive = driveTrain.WHEEL_DISTANCE_FROM_CENTER_MM * radians * driveTrain.COUNTS_PER_MM_DRIVE;

		int leftTarget = (int)(currentLeft - countsToDrive);
		int rightTarget = (int)(currentRight + countsToDrive);

		moveDriveMotorsByEncoder(leftTarget, rightTarget, speed, timeout);
	}

	private void moveDriveMotorsByEncoder(int leftTarget, int rightTarget, double speed, double timeout) {
		driveTrainEncoderActive = true;
		driveTrain.leftDrive.setTargetPosition(leftTarget);
		driveTrain.rightDrive.setTargetPosition(rightTarget);

		driveTrain.setLeftRightMode(DcMotor.RunMode.RUN_TO_POSITION);
		driveTrain.setLeftRightPower(speed);

		runtime.reset();
	}

	/**
	 * True means that it should keep going
	 * @return should the drive loop continue and the motors keep driving, based on if motors are moving
	 * and if the opmode is active
	 */
	protected boolean driveEncoderLoop () {
		telemetry.addData("left", driveTrain.leftDrive.getPower());
		telemetry.addData("right", driveTrain.rightDrive.getPower());
		telemetry.addData("left","%7d/%7d", driveTrain.leftDrive.getCurrentPosition(),
				driveTrain.leftDrive.getTargetPosition());
		telemetry.addData("right", "%7d/%7d", driveTrain.rightDrive.getCurrentPosition(),
				driveTrain.rightDrive.getTargetPosition());
		telemetry.update();
		opMode.idle();
		if (opMode.opModeIsActive() && /*runtime.milliseconds() < timeout &&*/
				(driveTrain.leftDrive.isBusy() || driveTrain.rightDrive.isBusy())) {
			return true;
		}else {
			driveComplete();
			return false;
		}
	}

	private void driveComplete () {
		driveTrain.setLeftRightPower(0);
		driveTrain.setLeftRightMode(DcMotor.RunMode.RUN_USING_ENCODER);
		telemetry.addLine("movement complete");
		telemetry.update();
		driveTrainEncoderActive = false;
	}



	/**LANDERVATOR STUFF**/
	protected void setLanderExtensionSpeed(double speed) {
		if (landervator.extensionMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) &&
				landervator.extensionMotor.getCurrentPosition() < landervator.MAX_EXTENDED_COUNTS) {
			landervator.extensionMotor.setPower(speed);
		}
		landervator.testMotorLimit(landervator.extensionMotor, landervator.MAX_EXTENDED_COUNTS);
		telemetry.addData("Landervator Pos", landervator.extensionMotor.getCurrentPosition());
	}

	protected void setLanderPitchSpeed(double speed) {
		if (landervator.pitchMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) &&
				landervator.pitchMotor.getCurrentPosition() < landervator.MAX_PITCH_COUNTS) {
			landervator.pitchMotor.setPower(speed);
			/*TODO when cratervator is made, look for something to move
				TODO it out of the way as this pitches up*/
		}
		landervator.testMotorLimit(landervator.pitchMotor, landervator.MAX_PITCH_COUNTS);
		telemetry.addData("Landervator pitchMotor", landervator.pitchMotor.getCurrentPosition());
	}

	protected void extendLanderToPos(int pos) {
		double speed = 1;
		landervator.extensionMotor.setTargetPosition(pos);
		landervator.extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		landervator.extensionMotor.setPower(speed);
		landervator.encoderMovementActive = true;
	}

	protected void pitchLanderToPos(int pos) {
		double speed = 1;
		landervator.pitchMotor.setTargetPosition(pos);
		landervator.pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		landervator.pitchMotor.setPower(speed);
		landervator.encoderMovementActive = true;
	}
}
