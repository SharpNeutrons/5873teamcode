package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareDTBot {

	protected FtcDashboard dashboard;
	protected MultipleTelemetry telemetry;

	private DriveTrain5873 driveTrain;
	private Landervator landervator;
	private SensoryManagement sensoryManagement;

	private AutoPilot autoPilot;


	/*Begin Motor Declaration*/
//	protected DcMotor linearExtension;
//	protected DcMotor cartMotor;
//	protected DcMotor intakeMotor;
//
//	/*Begin Servo Declaration*/
//	protected Servo intakeTwist;
	//Todo decide on other servos, such as ones to move from queue to cart

	/*Begin Sensor Declaration*/
	//Todo decide on sensors to use, such as Imu, openCV, color, ODS

	/*Begin Non Hardware Variable Declaration*/
	HardwareMap hwMap = null;
	public static final double HD_40_COUNTS_PER_REV = 2240;
	//Todo add things like elevator distances and servo mins and maxes

	protected HardwareDTBot () {

	}

	protected void init (OpMode5873 opMode) {
		hwMap = opMode.hardwareMap;

		dashboard = FtcDashboard.getInstance();
		telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

		driveTrain = new DriveTrain5873();
		driveTrain.init(opMode, hwMap, telemetry);

		landervator = new Landervator();
		landervator.init(opMode, hwMap, telemetry);

		//sensoryManagement = new SensoryManagement();
		//sensoryManagement.init(opMode, hwMap);


		//autoPilot.init(driveTrain, sensoryManagement);
	}

	protected void drive (double left, double right) {
		this.driveTrain.leftDrive.setPower(left);
		this.driveTrain.rightDrive.setPower(right);
	}

	protected void driveDistance (double distance, double speed, double timeout) {
		this.driveTrain.driveEncoderDistance(distance, speed, timeout);
	}

	protected void rotateDegrees (double degrees, double speed, double timeout) {
		this.driveTrain.rotateByEncoders(degrees, speed, timeout);
	}

	protected void setLandervatorExtensionSpeed(double speed) {
		landervator.setExtensionSpeed(speed);
	}

	protected void setLandervatorPitchSpeed(double speed) {
		landervator.setPitchSpeed(speed);
	}

	protected void setLandervatorExtensionPos(int pos) {
		landervator.extendToPos(pos);
	}

	protected void setLandervatorPitchPos(int pos) {
		landervator.pitchToPos(pos);
	}

	protected void unlockLandervator () {
		landervator.moveLock(Landervator.LOCK_POS.OPENED);
	}

	protected void lockLandervator () {
		landervator.moveLock(Landervator.LOCK_POS.LOCKED);
	}

	protected void updateTelemetry () {
		this.telemetry.update();
	}

	protected void loop () {
		landervator.runEncoderLoop();
	}

}
