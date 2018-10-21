package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name="Rotate")
public class AutoDriveTesting extends LinearOpMode {

	HardwareDTBot robot = new HardwareDTBot();

	public static double degrees = 0;
	public static double timeout = 5000;
	public static double speed = 0.5;

	@Override
	public void runOpMode() {
		robot.init(this);

		waitForStart();

		robot.driveTrain.rotateByEncoders(degrees, speed, timeout);
		//robot.driveTrain.driveEncoderDistance(914.4, 0.5, 5000);
	}

}
