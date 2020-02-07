/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootCommand extends Command {


  public ShootCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.kShooter);
    // eg. requires(chassis);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    if (!Robot.kShooter.shootStatus)
    {
      SmartDashboard.putBoolean("ShooterOn", false);
      Robot.kShooter.shootOut(0);
      if(Robot.oi.getPs4ShooterButton())
      {
        Robot.kShooter.shootStatus = true;
      }
    }
    else if (Robot.kShooter.shootStatus)
    {
      SmartDashboard.putBoolean("ShooterOn", true);
      Robot.kShooter.shootOut(-1.0);
      if(Robot.oi.getPs4ShooterButton())
      {
        Robot.kShooter.shootStatus = false;
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
    Robot.kShooter.stopMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.kShooter.stopMotors();
  }
}
