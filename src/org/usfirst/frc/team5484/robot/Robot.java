package org.usfirst.frc.team5484.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends SampleRobot {
	AHRS ahrs;
	RobotDrive robotDrive;
	CameraServer camServer;

	// Channels for the wheels
	final int kFrontLeftChannel = 9;
	final int kRearLeftChannel = 8;
	final int kFrontRightChannel = 7;
	final int kRearRightChannel = 6;

	// The channel on the driver station that the joystick is connected to
	final int kJoystickChannel = 0;

	Joystick stick = new Joystick(kJoystickChannel);

	public Robot() {
		robotDrive = new RobotDrive(kFrontLeftChannel, kRearLeftChannel, kFrontRightChannel, kRearRightChannel);
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true); 
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true); 
		// Add Camera Code to show on SmartDashboard
		camServer = CameraServer.getInstance();
	    camServer.startAutomaticCapture();	
		robotDrive.setExpiration(0.1);
		try {
			ahrs = new AHRS(SPI.Port.kMXP);
		}
		catch(RuntimeException ex)
		{
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
	}

	/**
	 * Runs the motors with Mecanum drive.
	 */
	@Override
	public void operatorControl() {
		robotDrive.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			double x = stick.getX();
			double y = stick.getY();
			double z = stick.getZ();
			double t = stick.getThrottle();
			t = (-(t+1)/2)+1;
			x = x*t;
			y = y*t;
			z = -(z*t);
			SmartDashboard.putDouble("X axis: ", x);
			SmartDashboard.putDouble("Y axis: ", y);
			SmartDashboard.putDouble("z axis: ", z);
			SmartDashboard.putDouble("Throttle: ", Math.round(t*100));
			SmartDashboard.putDouble("navX Angel: ", ahrs.getAngle());
			
			if(stick.getTrigger())
			{
				ahrs.reset();
			}
			try {
				robotDrive.mecanumDrive_Cartesian(x, y, z, ahrs.getAngle());
			}
			catch(RuntimeException ex) {
				DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
			}
		}
			
			

			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
	}
}
