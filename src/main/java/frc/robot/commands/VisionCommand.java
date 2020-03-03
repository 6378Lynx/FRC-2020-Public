/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

public class VisionCommand extends CommandBase {
  /**
   * Creates a new VisionCommand.
   */
  private double[] pose;
  private DriveSubsystem drive;
  public Joystick joystick;
 
  
  double[] defaultValue = new double[3];
  private NetworkTableInstance table;
  private NetworkTable cameraTable;
  private String cameraName = "Logitech";
  double curr_X_Distance;
  double curr_Y_Distance;
  double curr_angle_offset;

  double camera_distance = 0.5; //m
  double hypot_offset = 0.0;
  double latency;
  public VisionCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    joystick = new Joystick(1);
    table = NetworkTableInstance.getDefault();
    cameraTable = table.getTable("chameleon-vision").getSubTable(cameraName);
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void periodic() {
    System.out.println(cameraTable.getEntry("driver_mode").getType());
    pose = cameraTable.getEntry("targetPose").getDoubleArray(defaultValue);

  }

  public void execute() {
    pose = table.getEntry("targetPose").getDoubleArray(defaultValue);
    latency = cameraTable.getEntry("latency").getDouble(0.0);
    curr_X_Distance = pose[0];
    curr_Y_Distance = pose[1];
    curr_angle_offset = pose[2];
    //System.out.println(curr_X_Distance);
    if(joystick.getRawButton(0)){
      driveRobot();
    }
    SmartDashboard.putNumber("X offset", curr_X_Distance);
    SmartDashboard.putNumber("Y offset", curr_Y_Distance);
    SmartDashboard.putNumber("Angle offset", curr_angle_offset);
    SmartDashboard.putNumber("Latency ", latency);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
  public double x_offset(){
    return curr_X_Distance;
  }

  public double y_offset(){
    return curr_Y_Distance;
  }

  public double angle_offset(){
    return curr_angle_offset;
  }

  public double distance_hypotensuse(){
    return Math.hypot(curr_X_Distance, curr_Y_Distance) - camera_distance - hypot_offset;
  }

  public void driveRobot(){
    drive.arcadeDrive(distance_hypotensuse(), 0);
    drive.arcadeDrive(0, angle_offset());
  }

}
