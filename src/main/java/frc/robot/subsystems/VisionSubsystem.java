/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
  /**
   * Creates a new VisionSubsystem.
   */
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
  public VisionSubsystem() {
    joystick = new Joystick(1);
    table = NetworkTableInstance.getDefault();
    cameraTable = table.getTable("chameleon-vision").getSubTable(cameraName);
  }

  @Override
  public void periodic() {
    pose = cameraTable.getEntry("targetPose").getDoubleArray(defaultValue);
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
    System.out.println();
    SmartDashboard.putNumber("Latency ", latency);

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
