/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.ColorSensor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Commands.TestSparkDingusMotor;
import com.revrobotics.EncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.SparkMax;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.Spark;
/**
 * Add your docs here.
 */
public class Dingus extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands

  private CANSparkMax d_controlPanelMotor = new CANSparkMax(Constants.d_main, MotorType.kBrushless);
  private CANEncoder d_controlPanelEncoder = d_controlPanelMotor.getEncoder(EncoderType.kHallSensor, 42);
  
  public Dingus() {
  }

  public double getEncoderPosition(){
    return d_controlPanelEncoder.getPosition();
  }

  public double getEncoderVelocity()
  {
    return d_controlPanelEncoder.getVelocity();
  }

  public void printEncoderData()
  {
    SmartDashboard.putNumber("Dingus Encoder Position: ", getEncoderPosition());
    SmartDashboard.putNumber("Dingus Encoder Velocity: ", getEncoderVelocity());
  }

  public void driveDingusMotor(double power)
  {
    d_controlPanelMotor.set(power);
  }

  public void zeroDingusEncoder(){
    d_controlPanelEncoder.setPosition(0);
  }

  public void stopMotor()
  {
    d_controlPanelMotor.stopMotor();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(null);
  }
}
