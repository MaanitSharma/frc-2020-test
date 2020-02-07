/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

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
    //if(Robot.oi.getPs4ColorSensorButton()) //hold button to test
    //{
    //Robot.cSensor.printColorValue();
    //Robot.cSensor.retrieveColorInfo();

      /*Robot.cSensor.printRGB();
      Robot.cSensor.printProximity();
      Robot.cSensor.printIR();*/
    //}
    //else closes light :DDD
  }
    

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {

  }
}
