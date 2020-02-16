/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Subsystems.ColorSensor;

public class ColorSensorCommand extends Command {


  public ColorSensorCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.cSensor);
    // eg. requires(chassis);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.kDingus.printEncoderData();
    if(!Robot.cSensor.dDingusStatus) // function off
    {
      SmartDashboard.putBoolean("Dingus Functioning", false);
      Robot.cSensor.statusReset(); //idk useless
      Robot.kDingus.driveDingusMotor(0.0); //emergency stop, might be useless
      if(Robot.oi.getPs4ColorSensorButton()) 
      {
        Robot.cSensor.dDingusStatus = true;
      }
    }
    else if(Robot.cSensor.dDingusStatus) //function on
    {
      SmartDashboard.putBoolean("Dingus Functioning", true);
      Robot.cSensor.noFMSColorSpin();
      if(Robot.oi.getPs4ColorSensorButton()) 
      {
        Robot.cSensor.dDingusStatus = false; 
      }
    }
  }
    

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.kDingus.stopMotor();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.kDingus.stopMotor();
  }
}
