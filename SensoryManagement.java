package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class SensoryManagement {

	protected ColorSensorArray colorSensors;
	//protected OpticalDistanceSensor ods;
	protected BNO055IMU 			imu;
	//openCV stuff

	private LinearOpMode opMode;

	public SensoryManagement () {}

	protected void init (LinearOpMode _oM, HardwareMap hwm) {
		opMode = _oM;

		colorSensors = new ColorSensorArray();
		colorSensors.init(hwm);

		//ods 		= hwm.get(OpticalDistanceSensor.class, "ods");
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

	protected class ColorSensorArray {
		protected ColorSensor[][] sensors = new ColorSensor[2][2];

		protected ColorSensorArray () {}

		private void init (HardwareMap hwm) {
			//inits the color sensors with name colorSensor_i_j where i and j are the row/col # of the sensor
			for (int i = 0; i < sensors.length; i++) {
				for (int j = 0; j < sensors[i].length; j++) {
					sensors[i][j] = hwm.get(ColorSensor.class, "colorSensor_" + i +"_" + j);
				}
			}
		}

		protected double getSaturation (int i, int j) {
			try {
				ColorSensor s = sensors[i][j];
				float[] hsv = new float[3];
				Color.RGBToHSV(s.red(), s.green(), s.blue(), hsv);
				return hsv[1];
			} catch (Exception e) {
				Log.e("USER ERROR", "Give value between 0 and " + sensors[0].length);
				return 0;
			}
		}
	}
}
