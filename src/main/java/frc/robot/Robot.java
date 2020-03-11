/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.VisionCommand;
import frc.robot.subsystems.VisionSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //private Command m_autonomousCommand;
  private Command m_visionCommand;  public Joystick joystick;
 
  
  double[] defaultValue = new double[3];
  private NetworkTableInstance table;
  private NetworkTable cameraTable;
  public NetworkTableEntry isDriverMode;
  private String cameraName = "Logitech";
  double curr_X_Distance;
  double curr_angle_offset;
  double distance;
  double area_at_1_meter = 145.15;
  double currArea;

  double camera_distance = 0.5; //m
  double hypot_offset = 0.0;
  double latency;

  private RobotContainer m_robotContainer;
  private Solenoid solenoid = new Solenoid(0);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    joystick = new Joystick(1);
    table = NetworkTableInstance.getDefault();
    cameraTable = table.getTable("chameleon-vision").getSubTable(cameraName);
    isDriverMode = cameraTable.getEntry("driver_mode");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //if(joyst)

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // schedule the autonomous command (example)
    if (m_visionCommand != null) {
      m_visionCommand.schedule();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    solenoid.set(true);
    if(joystick.getRawButton(2)){
      latency = cameraTable.getEntry("latency").getDouble(0.0);
      curr_angle_offset = cameraTable.getEntry("targetYaw").getDouble(0.0);
      currArea = cameraTable.getEntry("targetFittedWidth").getDouble(0.0); 
      distance = 1 / (currArea * (double)(1 / (area_at_1_meter)  )  );
      //System.out.println(curr_X_Distance);
      SmartDashboard.putNumber("X offset", distance);
      SmartDashboard.putNumber("Angle offset", curr_angle_offset);
      SmartDashboard.putNumber("Latency ", latency);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
