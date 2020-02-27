/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ColourSensor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

/**
 * An example command that uses an example subsystem.
 */
public class ColourSensorCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ColourSensor m_subsystem;
  private String gameData;
  private String currentColour;
  private WPI_VictorSPX controlPanelMotor;
  DifferentialDrive drive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ColourSensorCommand(ColourSensor subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    controlPanelMotor = new WPI_VictorSPX(DriveConstants.redLineMotorPort);
    controlPanelMotor.set(ControlMode.PercentOutput, 0);
    drive = new DifferentialDrive(controlPanelMotor, controlPanelMotor);
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gameData = DriverStation.getInstance().getGameSpecificMessage();
  }

  public void rotateToColour(String targetColour){
    while(!currentColour.equals(targetColour)){
      drive.arcadeDrive(0.1, 0);
      currentColour = m_subsystem.getColor();
    }
  }

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentColour = m_subsystem.getColor();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
          if(currentColour.equals("Blue")){

          }else{
            rotateToColour("Blue");
          }
          break;
        case 'G' :
          if(currentColour.equals("Green")){

          }else{
            rotateToColour("Green");
          }
          break;
        case 'R' :
        if(currentColour.equals("Red")){

        }else{
          rotateToColour("Red");
        }
          break;
        case 'Y' :
          if(currentColour.equals("Yellow")){

          }else{
            rotateToColour("Yellow");
          }
          break;
        default :
          //This is corrupt data
          break;
      }
    } else {
      //Code for no data received yet
    }

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
