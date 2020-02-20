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
  private NetworkTableEntry pose;
  private NetworkTableEntry isDriverMode;;
  private NetworkTableInstance table;
  private NetworkTable cameraTable;
  private String cameraName = "";
  

  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault();
    cameraTable = table.getTable("chameleon-vision").getSubTable(cameraName);
    pose = cameraTable.getEntry("targetPose");
    isDriverMode = cameraTable.getEntry("driver_mode");
  }

  @Override
  public void periodic() {
    //System.out.println(yaw.getDouble(0.0));
  }
}
