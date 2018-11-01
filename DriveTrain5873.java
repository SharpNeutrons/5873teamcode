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

	protected final double WHEEL_RADUIS_MM = 50.8;
	protected final double COUNTS_PER_MM_DRIVE = /*HardwareDTBot.HD_40_COUNTS_PER_REV*/1120 / (WHEEL_RADUIS_MM * 2 * Math.PI);
	protected final double WHEEL_DISTANCE_FROM_CENTER_MM = 239.7;//237.015;//just a rough thing from the cad for now

	//protected final double STEER_WEIGHT = 0.75;
	//protected final double DRIVE_WEIGHT = 1 - STEER_WEIGHT;
	private ElapsedTime runtime = new ElapsedTime();

	protected DriveTrain5873() {
	}

	protected void init(LinearOpMode _oM, HardwareMap hwm, MultipleTelemetry _t) {
		leftDrive = hwm.get(DcMotor.class, "leftDrive");
		rightDrive = hwm.get(DcMotor.class, "rightDrive");

		leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
		rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

		setLeftRightMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		setLeftRightMode(DcMotor.RunMode.RUN_USING_ENCODER);

		leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		setLeftRightPower(0);
	}

	protected void setLeftRightPower(double power) {
		leftDrive.setPower(power);
		rightDrive.setPower(power);
	}

	protected void setLeftRightMode(DcMotor.RunMode mode) {
		leftDrive.setMode(mode);
		rightDrive.setMode(mode);
	}
}
