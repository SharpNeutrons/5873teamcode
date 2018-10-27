package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareDTBot {

	protected FtcDashboard dashboard;
	protected MultipleTelemetry telemetry;

	protected DriveTrain5873 driveTrain;
	protected Landervator landervator;
	protected SensoryManagement sensoryManagement;

	protected AutoPilot autoPilot;


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

		/*Begin Motor Setting*/

//		elevatorExtension	= hwMap.get(DcMotor.class, "elevatorExtension");
//		linearExtension		= hwMap.get(DcMotor.class, "linearExtension");
//		unfoldMotor			= hwMap.get(DcMotor.class, "unfoldMotor");
//		cartMotor			= hwMap.get(DcMotor.class, "cartMotor");
//		intakeMotor			= hwMap.get(DcMotor.class, "intakeMotor");
//
//		/*Begin Servo Setting*/
//		lazySuzan 	= hwMap.get(Servo.class, "lazySuzan");
//		intakeTwist	= hwMap.get(Servo.class, "intakeTwist");

		/*Begin Sensor Setting*/

		/*Begin Motor Info*/

//		elevatorExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		linearExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		unfoldMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		cartMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//		elevatorExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//		linearExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//		unfoldMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//		cartMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//		intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//		elevatorExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		linearExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		unfoldMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		cartMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//		autoPilot.init(driveTrain, sensoryManagement);
	}

}
