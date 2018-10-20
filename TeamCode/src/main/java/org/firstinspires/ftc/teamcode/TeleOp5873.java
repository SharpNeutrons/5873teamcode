package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DT unfortunate rule changes")
public class TeleOp5873 extends LinearOpMode {

	HardwareDTBot robot = new HardwareDTBot();

	@Override
	public void runOpMode() {
		robot.init(this);

		waitForStart();

		//robot.sensoryManagement.startImuIntegration();

		//automatically position the robot to line up to the crater

		while(opModeIsActive()) {
			teleopLoop();
		}
	}

	private void teleopLoop() {

		robot.driveTrain.teleOpSteeringStyle(gamepad1);

		if (gamepad1.left_bumper) {
			//move silver cart to scoring, gold cart to loading
		}else if (gamepad1.right_bumper) {
			//move gold cart to scoring, silver cart to loading
		}

		if (gamepad1.dpad_up) {
			//fold up the collector and elevator
		}else if (gamepad1.dpad_down) {
			//unfold the collector and elevator
		}

		//joystick control of the elevator

		robot.telemetry.update();
		idle();
	}
}