/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ColourSensor;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ExampleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ColourSensor m_subsystem;
  private String gameData;
  private String currentColour;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ColourSensor subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gameData = DriverStation.getInstance().getGameSpecificMessage();
  }

  public void rotateToColour(String targetColour){
    while(!currentColour.equals(targetColour)){

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

          }
          break;
        case 'G' :
          if(currentColour.equals("Green")){

          }else{
            
          }
          break;
        case 'R' :
        if(currentColour.equals("Red")){

        }else{
          
        }
          break;
        case 'Y' :
          if(currentColour.equals("Yellow")){

          }else{
            
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
