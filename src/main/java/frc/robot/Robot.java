// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  //Motor, Joysticks, encoder 
  private Spark leftTopmotor = new Spark(1); 
  private Spark rightTopmotor = new Spark(2); 
  private Spark leftBottommotor = new Spark(3);
  private Spark RightBottommotor = new Spark(4); 

  private Joystick joy1 = new Joystick(1);

  private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);

  privae final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;
  
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {

    SmartDashboard.putnumber("encoder value", encoder.get() * kDriveTick2Feet);
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    encoder.reset();
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();
  }

  final double kP = 0;
  final double kI = 0;
  final double kD = 0;
  final double iLimit = 0;
  

    double setpoint = 0;
    double errorSum = 0;
    double lastTimestamp = 0;
    double lastError = 0;
  
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
       //Joystick commands 
    if (joy1.getRawButton(1)) {
       setpoint = 0;
    } else if (joy1.getRawButton(2))
      setpoint = 0; 
  }
  // get sensor position
  double sensorPosition = encoder.get() * kDriveTick2Feet

  // calculations
  double error = setpoint - setposition
  double dt = Timer.getFPGATimestop() - lastTimestamp;

  if (Math.abs(error) < iLimit) {
  errorSum += error * dt;
  }

  double errorRate = (error - lastError) / dt;
  double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

  // output to motors
  leftTopmotor.set(outputSpeed);
  rightTopmotor.set(-outputSpeed);
  leftBottommotor.set(outputSpeed);
  RightBottommotor.set(-outputSpeed); 

  // update last - variable 
   lastTimestamp = Timer.getFPGATimestamp();
   lastError = error;
  }

  @Override
  public void autonomousExit() {}

  

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
