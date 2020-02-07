/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Commands.ShootCommand;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public boolean shootStatus = false; 

  public Shooter() {

  }

  private WPI_TalonSRX m_LeftShoot = new WPI_TalonSRX(Constants.s_left);
  private WPI_TalonSRX m_RightShoot = new WPI_TalonSRX(Constants.s_right);



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ShootCommand());
  }

  

  /*
  public void configTalons(WPI_TalonSRX tSrx)

  {

    //Tells the talon that the max output that it can give is between 1 and -1 which would mean full forward and full backward.
    tSrx.configPeakOutputForward(1,0);
    tSrx.configPeakOutputReverse(0,0);

    //Tells the talon that it should current limit its self so that we dont blow a 40Amp breaker.
    tSrx.configPeakCurrentLimit(40, 0);
    tSrx.enableCurrentLimit(true);
    tSrx.configContinuousCurrentLimit(40, 0);

    //The max output current is 40Amps for .25 of a second.
    tSrx.configPeakCurrentDuration(250, 0);

    //Tells the talon that it should only apply 12 volts (or less) to the motor.
    tSrx.configVoltageCompSaturation(12, 0);

  }
  */

  public void shootOut(double power){
    m_LeftShoot.set(ControlMode.PercentOutput, power);
    m_RightShoot.set(ControlMode.PercentOutput, power);

  }

  public void stopMotors(){
    m_LeftShoot.stopMotor();
    m_RightShoot.stopMotor();
  }

}
