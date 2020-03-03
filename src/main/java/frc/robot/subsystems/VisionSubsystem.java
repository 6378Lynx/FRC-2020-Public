/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;

public class VisionSubsystem extends SubsystemBase {
  private double[] pose;
  private NetworkTableEntry isDriverMode;
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
    isDriverMode = cameraTable.getEntry("driver_mode");
  }

  @Override
  public void periodic() {
    System.out.println(cameraTable.getEntry("driver_mode").getType());
    pose = cameraTable.getEntry("targetPose").getDoubleArray(defaultValue);
    latency = cameraTable.getEntry("latency").getDouble(0.0);
    curr_X_Distance = pose[0];
    curr_Y_Distance = pose[1];
    curr_angle_offset = pose[2];
    isDriverMode.setBoolean(joystick.getRawButton(0));
    //System.out.println(curr_X_Distance);
    SmartDashboard.putNumber("X offset", curr_X_Distance);
    SmartDashboard.putNumber("Y offset", curr_Y_Distance);
    SmartDashboard.putNumber("Angle offset", curr_angle_offset);
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


}
