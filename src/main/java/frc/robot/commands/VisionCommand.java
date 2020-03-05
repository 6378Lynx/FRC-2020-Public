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
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends CommandBase {
  /**
   * Creates a new VisionCommand.
   */
  private DriveSubsystem drive;
  private VisionSubsystem vision;
  public Joystick joystick;
  double distanceOffset;
  double angleOffset;
  
 
  
  public VisionCommand(VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    //drive = driveSubsystem;
    vision = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vision.setDriverMode(true);
    distanceOffset = vision.x_offset();
    angleOffset = vision.angle_offset();
    //Drive code here
    vision.setDriverMode(false);

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

  

}
