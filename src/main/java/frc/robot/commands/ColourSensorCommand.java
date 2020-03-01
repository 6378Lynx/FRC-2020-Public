/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ColourSensor;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ColourSensorCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ColourSensor m_subsystem;
  private String gameData;
  private String currentColour;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ColourSensorCommand(ColourSensor subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  public void rotateToColour(String targetColour){
    while(!currentColour.equals(targetColour)){

      currentColour = m_subsystem.getColor();
    }
  }

  public void rotateWheel(int rotations){
    int currRotations = 0;
    String lastColour = currentColour;
    //Every 8 colour changes indicates 1 rotation so for n rotations we would need n*7 colour changes
    while(currRotations != (rotations * 8)){
      //Rotate wheel
      if(!currentColour.equals(lastColour)){
        lastColour = currentColour;
        currRotations++;
      }
    }
  }

  
  public void positionControl(){
    currentColour = m_subsystem.getColor();
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0)){
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
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if(DriverStation.getInstance().getGameSpecificMessage().length() > 0){
      positionControl();
    }else{
      rotateWheel(4);
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
