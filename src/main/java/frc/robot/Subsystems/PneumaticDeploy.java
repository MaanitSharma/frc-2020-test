/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Commands.IntakeDeployCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;


/**
 * Add your docs here.
 */
public class PneumaticDeploy extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public boolean intakeDeployStatus = false;

  private DoubleSolenoid doubleSolenoid1;
  private DoubleSolenoid doubleSolenoid2;

  public PneumaticDeploy()
  {
    doubleSolenoid1 = new DoubleSolenoid(Constants.pcmCanId, Constants.portOpen1, Constants.portClose1);
    doubleSolenoid2 = new DoubleSolenoid(Constants.pcmCanId, Constants.portOpen2, Constants.portClose2);
  }

  public void forward()
  {
    doubleSolenoid1.set(DoubleSolenoid.Value.kForward);
    doubleSolenoid2.set(DoubleSolenoid.Value.kForward);
  }

  public void reverse()
  {
    doubleSolenoid1.set(DoubleSolenoid.Value.kReverse);
    doubleSolenoid2.set(DoubleSolenoid.Value.kReverse);
  }

  public void off()
  {
    doubleSolenoid1.set(DoubleSolenoid.Value.kOff);
    doubleSolenoid2.set(DoubleSolenoid.Value.kOff);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new IntakeDeployCommand());
  }
}
