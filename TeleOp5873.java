package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="5873 TeleOp", group="DT")
public class TeleOp5873 extends OpMode5873 {

	public TeleOp5873 () {
		super();
	}

	@Override
	public void runOpMode() {
		super.runOpMode();

		gp1Mode = Gamepad1Mode.DRIVE;

		waitForStart();

		//robot.sensoryManagement.startImuIntegration();

		//automatically position the robot to line up to the crater

		while(opModeIsActive()) {
			teleopLoop();
		}
	}

	private void teleopLoop() {

		this.drive();
		this.controlLandervator();

		//TODO add this stuff to either landervator or new class
//		if (gamepad1.left_bumper) {
//			//move silver cart to scoring, gold cart to loading
//		}else if (gamepad1.right_bumper) {
//			//move gold cart to scoring, silver cart to loading
//		}

		robot.updateTelemetry();
		robot.loop();
		idle();
	}

	private void drive () {
		double left = -gamepad1.left_stick_y;
		double right = -gamepad1.right_stick_y;
		robot.drive(left, right);
	}

	private void controlLandervator () {
		double extensionPower = -gamepad2.left_stick_y;
		double pitchPower = -gamepad2.left_stick_x / 3;

		robot.setLandervatorExtensionSpeed(extensionPower);
		robot.setLandervatorPitchSpeed(pitchPower);

		if (gamepad2.dpad_down) {
			robot.setLandervatorExtensionPos(0);
		} else if (gamepad2.dpad_up) {
			//robot.setLandervatorExtensionPos(0);//TODO make this the actual value
		}

		if (gamepad2.dpad_left) {
			robot.setLandervatorPitchPos(0);
		}else if (gamepad2.dpad_right) {
			//robot.setLandervatorPitchPos(0);//TODO make this the actual value
		}

		if (gamepad2.x) {
			robot.lockLandervator();
		}else if (gamepad2.b) {
			robot.unlockLandervator();
		}

	}
}
