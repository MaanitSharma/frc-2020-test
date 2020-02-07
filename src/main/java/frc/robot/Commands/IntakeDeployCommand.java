/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class IntakeDeployCommand extends Command {
  public IntakeDeployCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.kIntakeDeploy);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.kIntakeDeploy.reverse();
    //Robot.kIntakeDeploy.off();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!Robot.kIntakeDeploy.intakeDeployStatus)
    {
      SmartDashboard.putBoolean("IntakeDeploy", false);
      Robot.kIntakeDeploy.reverse(); 
      //Robot.kIntakeDeploy.off();
      if(Robot.oi.getPs4IntakeDeployButton())
      {
        Robot.kIntakeDeploy.intakeDeployStatus = true;
      }
    }
    else if (Robot.kIntakeDeploy.intakeDeployStatus)
    {
      SmartDashboard.putBoolean("IntakeDeploy", true);
      Robot.kIntakeDeploy.forward(); 
      if(Robot.oi.getPs4IntakeDeployButton())
      {
        Robot.kIntakeDeploy.intakeDeployStatus = false;
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
    Robot.kIntakeDeploy.off();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.kIntakeDeploy.off();
  }
}
