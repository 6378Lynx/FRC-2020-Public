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

public class VisionSubsystem extends SubsystemBase {
  private double[] pose;
  private NetworkTableEntry isDriverMode;
  public Joystick joystick;
 
  
  double[] defaultValue = new double[0];
  private NetworkTableInstance table;
  private NetworkTable cameraTable;
  private String cameraName = "";
  double curr_X_Distance;
  double curr_Y_Distance;
  double curr_angle_offset;
  

  public VisionSubsystem() {
    joystick = new Joystick(1);
    table = NetworkTableInstance.getDefault();
    cameraTable = table.getTable("chameleon-vision").getSubTable(cameraName);
    pose = table.getEntry("targetPose").getDoubleArray(defaultValue);
    isDriverMode = cameraTable.getEntry("driver_mode");
  }

  @Override
  public void periodic() {
    curr_X_Distance = pose[0];
    curr_Y_Distance = pose[1];
    curr_angle_offset = pose[2];
    isDriverMode.setBoolean(joystick.getRawButton(0));
    SmartDashboard.putNumber("X offset", curr_X_Distance);
    SmartDashboard.putNumber("Y offset", curr_Y_Distance);
    SmartDashboard.putNumber("Angle offset", curr_angle_offset);
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
