/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import static frc.robot.Constants.DriveConstants;

public class ColourSensor extends SubsystemBase {
  /**
   * Creates a new ColourSensor.
   */
  private final I2C.Port i2cPort;
  private final PWMVictorSPX victorSPX;
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher;
  private final Color blue;
  private final Color green;
  private final Color red;
  private final Color yellow;
  private Color detectedColor;
  private ColorMatchResult match;
  public static String colorString;

  public ColourSensor() {
    i2cPort = I2C.Port.kOnboard;
    victorSPX = new PWMVictorSPX(DriveConstants.redLineMotorPort);
    m_colorSensor = new ColorSensorV3(i2cPort);
    m_colorMatcher = new ColorMatch();
    blue = ColorMatch.makeColor(0.143, 0.427, 0.429);
    green = ColorMatch.makeColor(0.197, 0.561, 0.240);
    red = ColorMatch.makeColor(0.561, 0.232, 0.114);
    yellow = ColorMatch.makeColor(0.361, 0.524, 0.113);

    m_colorMatcher.addColorMatch(blue);
    m_colorMatcher.addColorMatch(green);
    m_colorMatcher.addColorMatch(red);
    m_colorMatcher.addColorMatch(yellow);

    detectedColor =  m_colorSensor.getColor();
    match = m_colorMatcher.matchClosestColor(detectedColor);
  }

  @Override
  public void periodic() {
        /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == blue) {
      colorString = "Blue";
    } else if (match.color == red) {
      colorString = "Red";
    } else if (match.color == green) {
      colorString = "Green";
    } else if (match.color == yellow) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }

  
  public String getColor(){
    return colorString;
  }

  public void rotateWheel(int rotations){
    int currRotations = 0;
    String lastColour = colorString;
    //Every 8 colour changes indicates 1 rotation so for n rotations we would need n*7 colour changes
    while(currRotations != (rotations * 8)){
      victorSPX.set(0.1);
      if(!colorString.equals(lastColour)){
        lastColour = colorString;
        currRotations++;
      }
    }
    victorSPX.set(0.0);
  }

  public void rotateToColour(String targetColour){
    victorSPX.set(0.1);
    while(!colorString.equals(targetColour)){
      colorString = getColor();
    }
    victorSPX.set(0.0);
  }

  public void positionControl(){
    colorString = getColor();
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0)){
        case 'B' :
          if(!colorString.equals("Blue")){
            rotateToColour("Blue");
          }
          break;
        case 'G' :
          if(!colorString.equals("Green")){
            rotateToColour("Green");
          }
          break;
        case 'R' :
          if(!colorString.equals("Red")){
            rotateToColour("Red");
          }
          break;
        case 'Y' :
          if(!colorString.equals("Yellow")){
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


}
