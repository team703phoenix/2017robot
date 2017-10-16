package org.usfirst.frc.team703.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot { // Create & initialize variables
	
	// Robot drive
	CANTalon left1, left2, left3, right1, right2, right3; // Drive motors
	double leftDrive, rightDrive; // Accel/decel drive variables
	RobotDrive robot;
	// Drive scalers: correction for curved driving in code
	final double LEFT_DRIVE_SCALER = 1;
	final double RIGHT_DRIVE_SCALER = 0.95;
	
	CANTalon winch1, winch2; // Winch motors
	
	Joystick controller = new Joystick(0); // Logitech F310 controller
	
	// Solenoids & controlling variables
	Solenoid gearShift = new Solenoid(0,2);
	Solenoid arm = new Solenoid(0,3);
	Solenoid clamp = new Solenoid(0,4);
	boolean highGear, armUp, clamped; // Booleans that control they're respective solenoids
	
	// Used for toggles to prevent rapid toggling
	boolean highGearToggle = true;
	boolean armUpToggle = true;
	boolean clampedToggle = true;
	
	// Sensors
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	final double GYRO_SCALE = 0.97; // The gyro isn't quite accurate, so we have to scale the value given to get an accurate measurement
	final double GYRO_CORRECTION_SCALE = 0.01; // Used during autonomous to aid the robot in driving straight
	
	Relay LEDs = new Relay(0); // LEDs (not currently on robot)
	
	// Autonomous
	boolean end = false; // If true, ends autonomous
	boolean button1, button2, button3, button4, button5; // Autonomous dashboard buttons
	
	@Override
	public void robotInit() {
		gyro.calibrate(); // Calibrate gyro to 0 degrees
		
		// Initialize cameras
		CameraServer.getInstance().addAxisCamera("axis-camera.local"); // Ethernet camera
		CameraServer.getInstance().startAutomaticCapture(); // USB Camera
		
		// Initialize drive motors
		left1 = new CANTalon(58);
		left2 = new CANTalon(53);
		left3 = new CANTalon(54);
		right1 = new CANTalon(55);
		right2 = new CANTalon(56);
		right3 = new CANTalon(57);
		
		// Set left2 and left3 as slaves as left1 (whatever left1 does, left2 and left3 also do)
		left2.changeControlMode(CANTalon.TalonControlMode.Follower);
		left2.set(left1.getDeviceID());
		left3.changeControlMode(CANTalon.TalonControlMode.Follower);
		left3.set(left1.getDeviceID());
		
		// Set right2 and right3 as slaves as right1 (whatever right1 does, right2 and right3 also do)
		right2.changeControlMode(CANTalon.TalonControlMode.Follower);
		right2.set(right1.getDeviceID());
		right3.changeControlMode(CANTalon.TalonControlMode.Follower);
		right3.set(right1.getDeviceID());
		
		// Initialize robot drive using left1 and right1 motors
		robot = new RobotDrive(left1,right1);
		
		// Initialize encoders
		left1.setFeedbackDevice(FeedbackDevice.EncRising);
		right1.setFeedbackDevice(FeedbackDevice.EncRising);
		
		// Initialize winch motors
		winch1 = new CANTalon(59);
		winch2 = new CANTalon(60);
		
		robot.setSafetyEnabled(false); // This was done to prevent errors that weren't really an issue from popping up
		
		LEDs.set(Value.kReverse); // Turn the LEDs on (LEDs are not currently on the robot)
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		end = false; // When end is true, autonomous ends
		clamp.set(false); // Close the clamp
		
		// Get autonomous selection from the dashboard
		button1 = SmartDashboard.getBoolean("DB/Button 1",false); // Left path
		button2 = SmartDashboard.getBoolean("DB/Button 2",false); // Center path
		button3 = SmartDashboard.getBoolean("DB/Button 3",false); // Right path
		button4 = SmartDashboard.getBoolean("DB/Button 4",false); // Baseline cross
		button5 = SmartDashboard.getBoolean("DB/Button 5",false); // Testing (used if we need to test an autonomous function easily)
	}

	/**
	 * This function is called periodically during autonomous
	 */	@Override
	public void autonomousPeriodic() {
		if (!end) {
			// driveForward and driveBackward are measured in inches
			// turnLeft and turnRight are measured in degrees
			// openClamp has a parameter that tells it how long to wait before to performing the next operation
			// (closeClamp does not have the same parameter)
			
			if (button1) { // If button1 is pressed
				System.out.println("Left path selected.");
				
				driveForward(98);
				turnRight(52);
				driveForward(35);
				openClamp(1);
				driveBackward(35);
				closeClamp();
				turnLeft(52);
				driveForward(72);
				
			} else if(button2) { // If button2 is pressed
				System.out.println("Center path selected.");
				
				driveForward(82.5);
				openClamp(1);
				driveBackward(24);
				closeClamp();
				
			} else if(button3) { // If button3 is pressed
				System.out.println("Right path selected.");
				
				driveForward(98);
				turnLeft(52);
				driveForward(35);
				openClamp(1);
				driveBackward(35);
				closeClamp();
				turnRight(52);
				driveForward(72);
				
			} else if(button4) { // If button4 is pressed
				System.out.println("Baseline cross selected.");
				
				driveForward(132);
				
			} else if(button5) { // If button5 is pressed
				System.out.println("Testing mode selected. The robot will now move forward until autonomous is disabled.");
				
				while(!end){
					
					left1.set(-0.5);
					right1.set(0.5);
					
					end = !isAutonomous() || !isEnabled();
				}
				
			}
		end = true; // Ends autonomous when the required operation is completed (otherwise, it will just repeat)
		}
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		// Stop the robot
		left1.set(0);
		right1.set(0);
		
		// Reset the encoders
		left1.setEncPosition(0);
		right1.setEncPosition(0);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		// Gear Shift (high gear/low gear)
		// Button: Right trigger
		if (controller.getRawAxis(3) > 0.9 && highGearToggle) {
			highGearToggle = false;
			highGear = !highGear;
		} else if(controller.getRawAxis(3) <= 0.9)
			highGearToggle = true;
		
		// Arm Control (raise/lower)
		// Button: Left bumper
		if (controller.getRawButton(5) && armUpToggle) {
			armUpToggle = false;
			armUp = !armUp;
		} else if(!controller.getRawButton(5))
			armUpToggle = true;
		
		// Clamp Control (open/close)
		// Button: Right bumper
		if (controller.getRawButton(6) && clampedToggle) {
			clampedToggle = false;
			clamped = !clamped;
		} else if(!controller.getRawButton(6))
			clampedToggle = true;

		// Winch control
		// Button: Left trigger
		if (controller.getRawAxis(2) > 0.9) {
			winch1.set(-1);
			winch2.set(-1);
		} else {
			winch1.set(0);
			winch2.set(0);
		}

		// Update the solenoids based on their respective boolean values
		gearShift.set(highGear);
		arm.set(armUp);
		clamp.set(clamped);
		
		// Turn off the LEDs if the button is pressed
		// Button: Back
		if (controller.getRawButton(7)) {
			LEDs.set(Value.kOff);
		} else {
			//kReverse makes it turn on. Why? I have no clue.
			LEDs.set(Value.kReverse);
		}
		
		// Left drive
		// If the robot is going backward, flip the drive scaler (so that the robot drives straight backwards as well)
		if (controller.getRawAxis(1) < 0) { // The controller's analog sticks are inverted, so < 0 means going forward
			left1.set(LEFT_DRIVE_SCALER * controller.getRawAxis(1));
		} else {
			left1.set(RIGHT_DRIVE_SCALER * controller.getRawAxis(1));
		}
		
		// Right drive
		// If the robot is going backward, flip the drive scaler (so that the robot drives straight backwards as well)
		if (controller.getRawAxis(5) < 0) { // The controller's analog sticks are inverted, so < 0 means going forward
			right1.set(-RIGHT_DRIVE_SCALER * controller.getRawAxis(5));
		} else {
			right1.set(-LEFT_DRIVE_SCALER * controller.getRawAxis(5));
		}
	}
		

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
	public static double accelDecel(double joyValue, double previous, double accRate){
		// ACCEL-DECEL IS CURRENTLY DEPRECATED
		
		/*
		 * Accel-Decel uses linear acceleration to easily change speeds without any bumps.
		 * The parameters include the joystick value (or the desired speed, whatever that may be),
		 * the previous drive speed to add on to, and the acceleration rate, what is added to the previous drive
		 * to create the ramping effect. Changing the acceleration rate when invoking the method will make the
		 * robot speed up and slow down at a faster rate.
		 * 
		 * Every time the teleopPeriodic code loops (around every 20 ms), this function is called to accelerate the
		 * robot and update the drive output accordingly. The acceleration effect is achieved by adding the acceleration
		 * rate amount to the previous drive to speed it up slightly. Since this function is called so often, this results
		 * in a smooth acceleration.
		 */
		
		double drive;
		
		// Calculate drive
		double joy = -joyValue; // Flip joyValue since the analog sticks on the controller are naturally inverted
		if (joy > 0.1 || joy < -0.1) { // If joy is between -0.1 and 0.1, don't move
			if ((joy - 0.03) > previous) { // If the joy value is greater than the current drive, start accelerating
				//Acceleration
				drive = previous + accRate; // Increment the drive by whatever the acceleration rate is to speed up gradually
			} else if((joy + 0.03) < previous) {
				//Deceleration
				drive = previous - accRate; // Decrement the drive by whatever the acceleration rate is to slow down gradually
			} else {
				drive = previous; // If the drive speed is in range of the desired speed, don't bother updating it
			}
		} else {
			drive = 0; // If the joystick is in range of 0, don't move the robot. This prevents small bumps from affecting the robot.
		}
		
		return drive;
	}

	public static double distanceToTicks(double distance) {
		
		/*
		 * Distance-to-ticks is used by the different drive functions used during autonomous.
		 * The only parameter needed is the distance, and the output is the amount of encoder
		 * ticks needed to cover that distance. A few constants, which vary from robot to robot,
		 * are needed to calculate the amount of ticks needed to cover a distance: the wheel circumference
		 * and the ticks per rotation (which can be found on the encoder specification sheet).
		 * 
		 * The conversion works by first converting the distance to the number of full wheel rotations
		 * required to cover that distance, then from rotations required to the amount of encoder ticks
		 * needed to cover the distance.
		 * 
		 * The amount of rotations required is calculated by taking the distance and dividing by the wheel
		 * circumference (which is equal to the distance covered in one rotation). In this formula,
		 * the circumference is calculated from the wheel diameter, as it is easier to measure a wheel's diameter
		 * than it's circumference.
		 * 
		 * The amount of ticks required is calculated by taking the number of rotations required and multiplying it
		 * by the amount of encoder ticks per rotation. 
		 */
		
		double output;
		
		final double WHEEL_DIAMETER = 4.25;
		final int TICKS_PER_ROTATION = 960;
		
		output = distance / (WHEEL_DIAMETER * Math.PI) * TICKS_PER_ROTATION;
		
		return output;
	}
	
	public void driveForward(double distance){
		
		/*
		 * The drive forward function drives forwards for a certain distance.
		 * 
		 * It works by converting the distance entered to ticks
		 * (via the distanceToTicks) function, resetting the encoders, and driving forward
		 * until the encoder value being read from the robot is greater than or equal to
		 * the desired amount of ticks (so, the desired distance).
		 * 
		 * In this process, the gyro, multiplied by the gyro correction angle, steers the
		 * robot straight if it ever curves.
		 * 
		 * In any part of the code, if end is triggered (meaning that the robot is either
		 * no longer enabled or no longer in autonomous mode), the process will stop and
		 * give control back to the autonomousPeriodic method, which will then end autonomous.
		 */
		
		// Convert distance to ticks
		double parameters = distanceToTicks(distance);
		
		// Reset encoders
		left1.setEncPosition(0);
		right1.setEncPosition(0);
		
		// Reset gyro (used to help the robot drive straight)
		gyro.reset();
		
		// Wait until encoders have reset to start moving
		// Simply ending when the encoder position was 0 created an infinite loop, as the encoders
		// are very sensitive and would often have some error (this is why any value under 1000 is
		// accepted).
		while ((left1.getEncPosition() > 1000 || right1.getEncPosition() > 1000) && !end)
			end = !isAutonomous() || !isEnabled();
		
		// Drive forward at 75% speed
		while (left1.getEncPosition() < parameters && right1.getEncPosition() < parameters && !end) {
			left1.set(-1 * ((0.75 * LEFT_DRIVE_SCALER) - (gyro.getAngle() * GYRO_CORRECTION_SCALE)));
			right1.set((0.75 * RIGHT_DRIVE_SCALER) + (gyro.getAngle() * GYRO_CORRECTION_SCALE));
			end = !isAutonomous() || !isEnabled();
		}
		robot.drive(0,0); // Stop the robot once the while loop ends (meaning it reached it's destination)
	}
	
	public void driveBackward(double distance){
		
		/*
		 * The drive backward function drives backwards for a certain distance.
		 * 
		 * It works by converting the distance entered to ticks
		 * (via the distanceToTicks) function, resetting the encoders, and driving backward
		 * until the encoder value being read from the robot is greater than or equal to
		 * the desired amount of ticks (so, the desired distance).
		 * 
		 * Even though the robot is going backwards, the encoder values are still positive
		 * 
		 * In this process, the gyro, multiplied by the gyro correction angle, steers the
		 * robot straight if it ever curves.
		 * 
		 * In any part of the code, if end is triggered (meaning that the robot is either
		 * no longer enabled or no longer in autonomous mode), the process will stop and
		 * give control back to the autonomousPeriodic method, which will then end autonomous.
		 */
		
		// Convert distance to ticks
		double parameters = distanceToTicks(distance);
		
		// Reset encoders
		left1.setEncPosition(0);
		right1.setEncPosition(0);
		
		// Reset gyro
		gyro.reset();
		
		// Wait until encoders have reset to start moving
		// Simply ending when the encoder position was 0 created an infinite loop, as the encoders
		// are very sensitive and would often have some error (this is why any value under 1000 is
		// accepted).
		while ((left1.getEncPosition() > 1000 || right1.getEncPosition() > 1000) && !end)
			end = !isAutonomous() || !isEnabled();
		
		// Drive backward at 75% speed
		while (left1.getEncPosition() < parameters && right1.getEncPosition() < parameters && !end) {
			left1.set((0.75 * RIGHT_DRIVE_SCALER) - (gyro.getAngle() * GYRO_CORRECTION_SCALE));
			right1.set(-1 * (0.75 * LEFT_DRIVE_SCALER) + (gyro.getAngle() * GYRO_CORRECTION_SCALE));
			end = !isAutonomous() || !isEnabled();
		}
		robot.drive(0,0); // Stop the robot once the while loop ends (meaning it reached it's destination)
	}
	
	public void turnRight(double angle){
		
		/*
		 * The turn right function turns the robot right to a certain angle.
		 * 
		 * It works by turning the robot right until the gyro is greater than or equal to
		 * the desired angle.
		 * 
		 * In any part of the code, if end is triggered (meaning that the robot is either
		 * no longer enabled or no longer in autonomous mode), the process will stop and
		 * give control back to the autonomousPeriodic method, which will then end autonomous.
		 */
		
		// Reset gyro to 0 degrees
		gyro.reset();
		
		// Turn right
		while (gyro.getAngle() * GYRO_SCALE < angle && !end) {
			left1.set(-0.6 * RIGHT_DRIVE_SCALER);
			right1.set(-0.6 * LEFT_DRIVE_SCALER);
			end = !isAutonomous() || !isEnabled();
		}
		robot.drive(0,0); // Stop the robot once the while loop ends (meaning it reached it's destination)
	}
	
	public void turnLeft(double angle){
		
		/*
		 * The turn left function turns the robot left to a certain angle.
		 * 
		 * It works by turning the robot right until the gyro is greater than or equal to
		 * the desired angle.
		 * 
		 * The angle is negated to fit the gyro's reading (turning counterclockwise gives a negative angle)
		 * 
		 * In any part of the code, if end is triggered (meaning that the robot is either
		 * no longer enabled or no longer in autonomous mode), the process will stop and
		 * give control back to the autonomousPeriodic method, which will then end autonomous.
		 */
		
		//Reset gyro
		gyro.reset();
		
		//Turn left
		while (gyro.getAngle() * GYRO_SCALE > -angle && !end) {
			left1.set(0.6 * LEFT_DRIVE_SCALER);
			right1.set(0.6 * RIGHT_DRIVE_SCALER);
			end = !isAutonomous() || !isEnabled();
		}
		robot.drive(0,0); // Stop the robot once the while loop ends (meaning it reached it's destination)
	}
	
	public void openClamp(int time){
		
		/*
		 * The open clamp function opens the clamp, and then wait a set amount of time before performing
		 * the next operation (which gives the pilot time to bring the gear up to the airship)
		 * 
		 * In any part of the code, if end is triggered (meaning that the robot is either
		 * no longer enabled or no longer in autonomous mode), the process will stop and
		 * give control back to the autonomousPeriodic method, which will then end autonomous.
		 */
		
		end = !isAutonomous() || !isEnabled();
		if(!end){
			// Open the clamp
			clamp.set(true);
			
			// Wait however long the parameter states
			try {
				Thread.sleep(1000 * time); // Measured in milliseconds
			} catch (InterruptedException ex) {
				Thread.currentThread().interrupt();
			}
		}
	}
	
	public void closeClamp(){
		
		/*
		 * The open clamp function closes the clamp without a wait time.
		 * 
		 * In any part of the code, if end is triggered (meaning that the robot is either
		 * no longer enabled or no longer in autonomous mode), the process will stop and
		 * give control back to the autonomousPeriodic method, which will then end autonomous.
		 */
		
		end = !isAutonomous() || !isEnabled();
		if(!end)
			// Close the clamp
			clamp.set(false);
	}
}












































































































































































































































































