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

public class VisionSubsystem extends SubsystemBase {
  private double[] pose;
  private NetworkTableEntry isDriverMode;
  double[] defaultValue = new double[0];
  private NetworkTableInstance table;
  private NetworkTable cameraTable;
  private String cameraName = "";
  double curr_X_Distance;
  double curr_Y_Distance;
  double curr_angle_offset;
  

  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault();
    cameraTable = table.getTable("chameleon-vision").getSubTable(defaultValue);
    pose = table.getEntry("targetPose").getDoubleArray(pose);
    isDriverMode = cameraTable.getEntry("driver_mode");
  }

  @Override
  public void periodic() {
    curr_X_Distance = pose[0];
    curr_Y_Distance = pose[1];
    curr_angle_offset = pose[2];
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

}
