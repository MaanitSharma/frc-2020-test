/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import frc.robot.*;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Commands.LimelightCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveCommand extends Command {

  Command limelightCommand;
  public DriveCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.kDrivetrain);
    // eg. requires(chassis);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.kDrivetrain.configEncodersForDrive();
    Robot.kDrivetrain.zeroEncoder();
    limelightCommand = new LimelightCommand();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Robot.kDrivetrain.driveJoystick();
    SmartDashboard.putBoolean("AutoAlignStats", Robot.limelight.autoAlignStatus);
    if(!Robot.limelight.autoAlignStatus) //if false then afk
    {
      Robot.kDrivetrain.driveJoystick();
      if(Robot.oi.getPs4AutoAdjustButton())
      {
        Robot.limelight.autoAlignStatus = true;
      }
    }
    else if(Robot.limelight.autoAlignStatus)
    {   
      limelightCommand.start();
      if(Robot.oi.getPs4AutoAdjustButton())
      {
        Robot.limelight.autoAlignStatus = false;
      }   
    }
    //Robot.limelight.updateLimeLightAutoDrive(3, -0.1, -0.1);
      //Robot.kDrivetrain.driveSlave(Robot.limelight.getLeftSpeedTune(), Robot.limelight.getRightSpeedTune());
      
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
