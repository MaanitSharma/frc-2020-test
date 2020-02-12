package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import frc.robot.Commands.ColorSensorCommand;
import frc.robot.Constants;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.EncoderType;
import frc.robot.Subsystems.Dingus;
import frc.robot.Robot;

public class ColorSensor extends Subsystem {

  private final I2C.Port i2cPort;
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher = new ColorMatch();
  // Type in the RGB value of each color at that distance to config the output
  private final Color kBlueTarget = ColorMatch.makeColor(0.226, 0.462, 0.333);
  private final Color kGreenTarget = ColorMatch.makeColor(0.3, 0.53, 0.176);
  private final Color kRedTarget = ColorMatch.makeColor(0.612, 0.31, 0.112);
  private final Color kYellowTarget = ColorMatch.makeColor(0.429, 0.478, 0.091);

  // private CANSparkMax d_controlPanelMotor;
  // private CANEncoder d_controlPanelEncoder;

  public boolean dDingusStatus = false;

  private boolean hasTargetColor = false;
  private boolean updatePrevColor = false;
  private int motorRotation = 0; // 0 = left, 1 = right
  private int targetColor;
  private int prevColor;
  private int currColor;
  private int incomingColor = 0;
  private int counter;

  public ColorSensor() {
    i2cPort = I2C.Port.kOnboard;
    // d_controlPanelMotor = new CANSparkMax(Constants.d_main,
    // MotorType.kBrushless);
    // d_controlPanelEncoder =
    // d_controlPanelMotor.getEncoder(EncoderType.kQuadrature, 4096);
    // d_controlPanelMotor.restoreFactoryDefaults();
    m_colorSensor = new ColorSensorV3(i2cPort);
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  public Color retrieveColor() {
    Color detectedColor = m_colorSensor.getColor();
    SmartDashboard.putNumber("R", detectedColor.red);
    SmartDashboard.putNumber("G", detectedColor.green);
    SmartDashboard.putNumber("B", detectedColor.blue);
    return detectedColor;
  }
  // For String Allocation
  /*
   * public String matchColor(Color detectedColor) { String colorString;
   * ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor); if
   * (match.color == kBlueTarget) colorString = "Blue"; else if (match.color ==
   * kRedTarget) colorString = "Red"; else if (match.color == kGreenTarget)
   * colorString = "Green"; else if (match.color == kYellowTarget) colorString =
   * "Yellow"; else colorString = "Unknown"; return colorString; }
   */

  public int matchColor(Color detectedColor) {
    int colorInt;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget)
      colorInt = 1; // Blue
    else if (match.color == kRedTarget)
      colorInt = 2;
    else if (match.color == kGreenTarget)
      colorInt = 3;
    else if (match.color == kYellowTarget)
      colorInt = 4;
    else
      colorInt = 1000; // Unknown Color
    return colorInt;
  }

  public void noFMSColorSpin() // return color string... counter variables etc... ready
  {
    if (!hasTargetColor) {
      targetColor = matchColor(retrieveColor());
      counter = 0;
      hasTargetColor = true;
      SmartDashboard.putNumber("targetColorInt", targetColor);
    }

    if (!updatePrevColor) {
      prevColor = matchColor(retrieveColor());
      updatePrevColor = true;
    }
    if (hasNewColor()) {
      updatePrevColor = false;
      if (currColor == targetColor)
        counter++;
    }
    if (counter < 6) // 3 times x 2 color block
    {
      if (motorRotation == 0) // default 0, can increase efficiency later on //need adjustment
      {
        Robot.kDingus.driveDingusMotor(0.1); // spinning power
      }
    } else {
      Robot.kDingus.driveDingusMotor(0.0);
      hasTargetColor = false;
      dDingusStatus = false;
    }

    SmartDashboard.putNumber("prevColorInt", prevColor);
    SmartDashboard.putNumber("currColorInt", currColor);
    SmartDashboard.putNumber("incomingColorInt", incomingColor);
    SmartDashboard.putNumber("currSpinCounterInt", counter / 2);
  }

  public boolean hasNewColor() {
    currColor = matchColor(retrieveColor());
    if (currColor == prevColor)
      return false;
    updateNextColor();
    if (currColor == incomingColor)
      return true;
    else {
      System.out.println("Skipped Color!");
      return false;
    }
  }

  /*public void updateNextColor() {
    if (motorRotation == 0) // disk spinning clockwise (to the right)
    {
      if (prevColor == 1)
        incomingColor = 4; // Prev is blue, incoming is Yellow
      else if (prevColor == 4)
        incomingColor = 2;// Prev is yellow, incoming is red
      else if (prevColor == 2)
        incomingColor = 3;// Prev is red, incoming is green
      else if (prevColor == 3)
        incomingColor = 1;// Prev is green, incoming is blue
    } else if (motorRotation == 1) // disk spinning counterclockwise (to the left)
    {
      if (prevColor == 1)
        incomingColor = 3;
      else if (prevColor == 3)
        incomingColor = 2;
      else if (prevColor == 2)
        incomingColor = 4;
      else if (prevColor == 4)
        incomingColor = 1;
    }
  }*/

  public void updateNextColor() //1 is blue, 2 is red, 3 is green, 4 is yellow
  {
    if(motorRotation == 0)
    {
      switch(prevColor) {
        case 1:
          incomingColor = 4;
          break;
        case 2:
          incomingColor = 3;
          break;
        case 3:
          incomingColor = 1;
          break;
        case 4:
          incomingColor = 2;
          break;
      }
    }
    else
    {
      switch(prevColor) {
        case 1:
          incomingColor = 3;
          break;
        case 2:
          incomingColor = 4;
          break;
        case 3:
          incomingColor = 2;
          break;
        case 4:
          incomingColor = 1;
          break;
      }
    }
  }

  /*
   * public void driveDingusMotor(double power) {
   * //d_controlPanelMotor.set(power); }
   */

  public void statusReset() {
    hasTargetColor = false;
    updatePrevColor = false;
  }

  /*
   * public void stopMotor() { d_controlPanelMotor.stopMotor(); }
   */

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(null);
    setDefaultCommand(new ColorSensorCommand());
  }
}
