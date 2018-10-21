package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class SensoryManagement {

	protected ColorSensor 			colorFront;
	protected ColorSensor 			colorBack;
	protected OpticalDistanceSensor ods;
	protected BNO055IMU 			imu;
	//openCV stuff

	private LinearOpMode opMode;

	public SensoryManagement () {}

	protected void init (LinearOpMode _oM, HardwareMap hwm) {
		opMode = _oM;

		colorFront	= hwm.get(ColorSensor.class, "colorFront");
		colorBack 	= hwm.get(ColorSensor.class, "colorBack");
		ods 		= hwm.get(OpticalDistanceSensor.class, "ods");
		imu 		= hwm.get(BNO055IMU.class, "imu");
		this.initIMU();

	}

	private void initIMU () {
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json";//TODO actually understand this see the calibration sample opmode
		parameters.loggingEnabled      = true;
		parameters.loggingTag          = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

		imu.initialize(parameters);
	}

	protected void startImuIntegration () {
		imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
	}
}
