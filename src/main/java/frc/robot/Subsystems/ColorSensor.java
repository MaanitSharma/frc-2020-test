package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import frc.robot.Commands.ColorSensorCommand;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor extends Subsystem {

  private final I2C.Port i2cPort;
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher = new ColorMatch();
  //Type in the RGB value of each color at that distance to config the output
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427,  0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private boolean hasCurrColor = false;

  public ColorSensor() {
    i2cPort = I2C.Port.kOnboard;
    m_colorSensor = new ColorSensorV3(i2cPort);
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  /*public double getIR() //useless
  {
    return m_colorSensor.getIR();
  }

  public double getRed() //useless
  {
    return m_colorSensor.getColor().red;
  }

  public double getGreen() //useless 
  {
    return m_colorSensor.getColor().green;
  }

  public double getBlue() //useless
  {
    return m_colorSensor.getColor().blue;
  }

  public int getProximity() //useless
  {
    return m_colorSensor.getProximity();
  }

  public void printColorValue() //useless
  {
    SmartDashboard.putNumber("IR", getIR());
    SmartDashboard.putNumber("Red", getRed());
    SmartDashboard.putNumber("Green", getGreen());
    SmartDashboard.putNumber("Blue", getBlue());
    SmartDashboard.putNumber("Proximity", getProximity());
  }*/

  /*public void retrieveColorInfo()
  {
    Color detectedColor = m_colorSensor.getColor();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) colorString = "Blue";
    else if (match.color == kRedTarget) colorString = "Red";
    else if (match.color == kGreenTarget) colorString = "Green";
    else if (match.color == kYellowTarget) colorString = "Yellow";
    else colorString = "Unknown";
    
    SmartDashboard.putNumber("R", detectedColor.red);
    SmartDashboard.putNumber("G", detectedColor.green);
    SmartDashboard.putNumber("B", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence); //similarity
    SmartDashboard.putString("Detected Color", colorString);
  } */

  public Color retrieveColor()
  {
    return m_colorSensor.getColor();
  }

  public String matchColor(Color detectedColor)
  {
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) colorString = "Blue";
    else if (match.color == kRedTarget) colorString = "Red";
    else if (match.color == kGreenTarget) colorString = "Green";
    else if (match.color == kYellowTarget) colorString = "Yellow";
    else colorString = "Unknown";
    return colorString;
  }
  

  public void noFMSColorSpin() //return color string... counter variables etc... ready
  {
    Color currColor = retrieveColor();

    
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ColorSensorCommand());
  }
}

