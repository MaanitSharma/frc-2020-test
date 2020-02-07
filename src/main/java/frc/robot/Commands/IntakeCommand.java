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

public class IntakeCommand extends Command {
  public IntakeCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    //requires(Robot.kIntake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //press the same mode once to activiate, 2nd time to stop, or press the opposite mode to activate the oppo. mode
    /*if(!Robot.kIntake.intakeStatus && Robot.kIntake.outtakeStatus) //intake false, outtake true
    {
      SmartDashboard.putBoolean("IntakeMode", false);
      SmartDashboard.putBoolean("OuttakeMode", true);
      Robot.kIntake.outtake(1);
      if(Robot.oi.getPs4IntakeOuttakeButton()) //if pressed outtake button
      {
        Robot.kIntake.outtakeStatus = false; //turn off... f & f
      }
      else if(Robot.oi.getPs4IntakeIntakeButton()) //if pressed intake button
      {
        Robot.kIntake.intakeStatus = true;
        Robot.kIntake.outtakeStatus = false; //switch to intake mode
      }
    }
    else if(Robot.kIntake.intakeStatus && !Robot.kIntake.outtakeStatus) //intake true, outtake false
    {
      SmartDashboard.putBoolean("IntakeMode", true);
      SmartDashboard.putBoolean("OuttakeMode", false);
      Robot.kIntake.intake(1);
      if(Robot.oi.getPs4IntakeIntakeButton()) //if pressed intake button
      {
        Robot.kIntake.intakeStatus = false; //turn off.. f & f
      }
      else if(Robot.oi.getPs4IntakeOuttakeButton()) //if pressed outtake button
      {
        Robot.kIntake.intakeStatus = false;
        Robot.kIntake.outtakeStatus = true; //switch to outtake mode
      }
    }
    else if(!Robot.kIntake.intakeStatus && !Robot.kIntake.outtakeStatus) //both false, stops
    {
      SmartDashboard.putBoolean("IntakeMode", false);
      SmartDashboard.putBoolean("OuttakeMode", false);
      Robot.kIntake.intake(0);
      if(Robot.oi.getPs4IntakeIntakeButton()) //if pressed intake button
      {
        Robot.kIntake.intakeStatus = true; //activates intake
      }
      else if(Robot.oi.getPs4IntakeOuttakeButton()) //if pressed outtake button
      {
        Robot.kIntake.outtakeStatus = true; //activates outtake
      }
    }
    //a case of t and t will never happen.*/
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //Robot.kIntake.stopMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //Robot.kIntake.stopMotors();
  }
}
