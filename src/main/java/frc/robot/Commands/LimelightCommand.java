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
import frc.robot.Commands.DriveCommand;

public class LimelightCommand extends Command {
  
  boolean finished;
  public LimelightCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.limelight);
    requires(Robot.kDrivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    finished = false;
    Robot.limelight.resetSpeedTune();
    Robot.kDrivetrain.initPID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.limelight.printToDashboard();
    //Robot.limelight.autoAngling(Robot.limelight.getXDegreeOffset());
    //Robot.limelight.autoDistancing(65, Robot.limelight.getYDegreeOffset());
    //Robot.limelight.updateLimeLightAutoDrive(Robot.limelight.getXDegreeOffset(), Robot.limelight.getYDegreeOffset(), 65);
    //Robot.kDrivetrain.driveSlave(Robot.limelight.getLeftSpeedTune() * 0.2 , Robot.limelight.getRightSpeedTune() * 0.2); //works in autoalign
    //Robot.kDrivetrain.driveSlave(Robot.limelight.getLeftSpeedTune() * 0.2, Robot.limelight.getRightSpeedTune() * 0.95);
    //obot.limelight.autoPIDDistancing(65.0, Robot.limelight.getYDegreeOffset());
    Robot.limelight.autoLimelightDrive(Robot.limelight.getXDegreeOffset(), Robot.limelight.getYDegreeOffset(), 100);
    //Robot.limelight.autoLimelightDrive(Robot.limelight.getXDegreeOffset(), Robot.limelight.getYDegreeOffset(), 65);

    finished = true;
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
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
