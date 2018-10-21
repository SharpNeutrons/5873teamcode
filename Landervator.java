package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Landervator {

	protected DcMotor extensionMotor;
	protected DcMotor pitch;

	private LinearOpMode opMode;
	private MultipleTelemetry telemetry;

	protected Landervator () {};

	protected void init (LinearOpMode _oM, HardwareMap hwm, MultipleTelemetry _t) {
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
	}

	protected void teleOpControl (Gamepad gp1, Gamepad gp2) {

	}

}