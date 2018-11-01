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





	/**DRIVING CONTROL**/

	protected void drive (double left, double right) {
		this.driveTrain.leftDrive.setPower(left);
		this.driveTrain.rightDrive.setPower(right);
	}

	/**
	 * This one should only be called if the opmode has a loop (like teleop) which calls the loop()
	 * function in this Hardware class
	 * @param distance
	 * @param speed
	 * @param timeout
	 */
	protected void driveDistance (double distance, double speed, double timeout) {
		autoPilot.driveEncoderDistance(distance, speed, timeout);
	}

	protected void driveDistInescapable (double dist, double speed, double timeout) {
		autoPilot.driveEncoderDistance(dist, speed, timeout);
		while (driveLoop()){}
	}

	/**
	 * This one should only be used if the opmode has a loop (like a teleop) which calls loop() in
	 * the hardware class
	 * @param degrees
	 * @param speed
	 * @param timeout
	 */
	protected void rotateDegrees (double degrees, double speed, double timeout) {
		autoPilot.rotateByEncoders(degrees, speed, timeout);
	}

	protected void rotateDegreesInescapable (double degrees, double speed, double timeout) {
		autoPilot.rotateByEncoders(degrees, speed, timeout);
		while(driveLoop()){}
	}

	private boolean driveLoop () {
		if (autoPilot.driveTrainEncoderActive) {
			return autoPilot.driveEncoderLoop();
		}
		return false;
	}






	/**LANDERVATOR CONTROL**/

	protected void setLandervatorExtensionSpeed(double speed) {
		autoPilot.setLanderExtensionSpeed(speed);
	}

	protected void setLandervatorPitchSpeed(double speed) {
		autoPilot.setLanderPitchSpeed(speed);
	}

	/**
	 * This one should only be used if the opmode has a loop (like a teleop) which calls loop() in
	 * the hardware class
	 */
	protected void setLandervatorExtensionPos(int pos) {
		autoPilot.extendLanderToPos(pos);
	}

	protected void setLandervatorExtensionPosInescapable(int pos) {
		autoPilot.extendLanderToPos(pos);
		while(landerLoop()){}
	}

	/**
	 * This one should only be used if the opmode has a loop (like a teleop) which calls loop() in
	 * the hardware class
	 */
	protected void setLandervatorPitchPos(int pos) {
		autoPilot.pitchLanderToPos(pos);
	}

	protected void setLandervatorPitchPosInescapable(int pos) {
		autoPilot.pitchLanderToPos(pos);
		while(landerLoop());
	}

	protected void unlockLandervator () {
		landervator.moveLock(Landervator.LOCK_POS.OPENED);
	}

	protected void lockLandervator () {
		landervator.moveLock(Landervator.LOCK_POS.LOCKED);
	}

	private boolean landerLoop () {
		if (landervator.encoderMovementActive) {
			return landervator.runEncoderLoop();
		}
		return false;
	}





	/**GENERAL FUNCTIONS**/

	protected void updateTelemetry () {
		this.telemetry.update();
	}

	protected void loop () {
		driveLoop();
		landerLoop();
	}




}
