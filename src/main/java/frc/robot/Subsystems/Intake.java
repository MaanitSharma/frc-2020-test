/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Intake()
  {

  }

  private WPI_TalonSRX m_Intake = new WPI_TalonSRX(Constants.i_main);

  public boolean intakeStatus = false;
  public boolean outtakeStatus = false;

  public void intake(double power){
    m_Intake.set(ControlMode.PercentOutput, -power);
  }

  public void outtake(double power){
    m_Intake.set(ControlMode.PercentOutput, power);
  }

  public void stopMotors()
  {
    m_Intake.stopMotor();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    //setDefaultCommand(new IntakeCommand());
  }
}
