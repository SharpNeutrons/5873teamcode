package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class OpMode5873 extends LinearOpMode {

	protected HardwareDTBot robot = new HardwareDTBot();
	public Gamepad1Mode gp1Mode;

	/**
	 * Only runs the common init sections for the opmodes
	 */
	@Override
	public void runOpMode() {
		robot.init(this);

		//robot.sensoryManagement.startImuIntegration();

	}

}
