package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class AutoPilot {

	private DriveTrain5873 driveTrain;
	//elevator management
	//collector
	private SensoryManagement sensoryManagement;

	public AutoPilot (){}

	protected void init (DriveTrain5873 _dT, /*elevator, collector*/ SensoryManagement _sM){
		driveTrain = _dT;
		//elevator
		//collector
		sensoryManagement = _sM;
	}

	/**Tells the robot to drive to a tape line (blue or red)
	 *
	 * @param speed 0 to 1 how fast the motors should go
	 * @param direction 1 for forward, -1 for backward
	 */
	protected void driveToLine (double speed, double direction) {
		speed = Range.clip(speed, 0, 1);

	}

}
