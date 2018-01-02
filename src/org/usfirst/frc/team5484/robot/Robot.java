package org.usfirst.frc.team5484.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends SampleRobot {
	RobotDrive robotDrive;

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
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true); // invert the
																	// left side
																	// motors
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true); // you may need
																// to change or
																// remove this
																// to match your
																// robot
		robotDrive.setExpiration(0.1);
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
			
			
			
			robotDrive.mecanumDrive_Cartesian(x, y, z, 0);

			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}
}
