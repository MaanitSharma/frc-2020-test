/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Commands.LimelightCommand;
import edu.wpi.first.networktables.NetworkTable; 
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  NetworkTable table;
  double v;
  double s;
  double x;
  double y;
  double area;
  double leftSpeedTune = 0;
  double rightSpeedTune = 0;

  final double iLimit = 1;
  double distanceErrorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;

  public boolean autoAlignStatus = false;

  public Limelight()
  {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void setPipeline(int pipeline)
  {
    this.table.getEntry("pipeline").setNumber(pipeline);
  }

  public void setLEDs(int state)
  {
    this.table.getEntry("ledMode").setNumber(state);
  }

  /*public void updateLimelightVariables()
  {
    v = table.getEntry("tv").getDouble(0.0); 
    s = table.getEntry("ts").getDouble(0.0); 
    x = table.getEntry("tx").getDouble(0.0); //horizontal degree off set from target
    y = table.getEntry("ty").getDouble(0.0); //vertical degree off set from target
    area = table.getEntry("ta").getDouble(0.0);
  }*/

  public double getXDegreeOffset()
  {
    return table.getEntry("tx").getDouble(0.0); //horizontal degree off set from target
  }

  public double getYDegreeOffset()
  {
    return table.getEntry("ty").getDouble(0.0); //vertical degree off set from target
  }

  public boolean hasValidTarget()
  {
    return table.getEntry("tv").getDouble(0.0) == 1; //1 == hasTarget, 0 == nope
  }

  public void printToDashboard()
  {
    //updateLimelightVariables();
    SmartDashboard.putBoolean("ValidTarget", hasValidTarget());
    SmartDashboard.putNumber("LimelightX", getXDegreeOffset());
    SmartDashboard.putNumber("LimelightY", getYDegreeOffset());
    //SmartDashboard.putNumber("LimelightArea", area);
  }

  public void updateLimeLightAutoDrive(double tx, double ty, double desiredDistance) //beast unleashed
  {
    double KpDistance = -0.18;
    double KpAim = -0.1;
    double cVTA = ty;//getYDegreeOffset(); //camera to vision target cneter y angle from cMA
    double currentDistance = (Constants.hOVT - Constants.hOL) / Math.tan(Math.toRadians(Constants.cMA + cVTA)); //d = (h2-h1) / tan(a1+a2)
    SmartDashboard.putNumber("currentDistance", currentDistance);
    double distanceError = currentDistance - desiredDistance;
    SmartDashboard.putNumber("desiredDistance", desiredDistance);
    SmartDashboard.putNumber("distanceError", distanceError);
    //double KpDistance = -0.1; double KpAim = -0.1;
    double distanceAdjust = KpDistance * distanceError;
    double minAimCommand = 0.05; //prevent robot from too little movement...cheaper way instead of intergration
    double headingError = -tx;//getXDegreeOffset();
    double steeringAdjust = 0.0;
    if (-headingError > 1.0)
    {
    	steeringAdjust = KpAim * headingError - minAimCommand;
    }
    else if (-headingError < 1.0)
    {
    	steeringAdjust = KpAim * headingError + minAimCommand;
    }
    
    leftSpeedTune -= steeringAdjust + distanceAdjust;
    rightSpeedTune += steeringAdjust + distanceAdjust;
    SmartDashboard.putNumber("leftSpeedTune", leftSpeedTune);
    SmartDashboard.putNumber("rightSpeedTune", rightSpeedTune);
  }

  public void autoAngling(double tx) //done! speed cap at 0.4
  {
    double kp_angling = -0.1;
    double minCommand = 0.05;
    double headingError = -tx;
    double steeringAdjust = 0.0;
    if(tx > 1) steeringAdjust = kp_angling*headingError - minCommand;
    else if(tx < 1) steeringAdjust = kp_angling * headingError + minCommand;
    
    leftSpeedTune -= steeringAdjust;
    rightSpeedTune += steeringAdjust;
    SmartDashboard.putNumber("leftSpeedTune", leftSpeedTune);
    SmartDashboard.putNumber("rightSpeedTune", rightSpeedTune);
  }

  public void autoDistancing(double desiredDistance, double ty) //skipped no time to test sadly :/
  {
    double kp_distancing = -0.25;
    double ki = -0.1;
    double kd = -0.1;

    
    double cVTA = ty;//getYDegreeOffset(); //camera to vision target cneter y angle from cMA
    double currentDistance = (Constants.hOVT - Constants.hOL) / Math.tan(Math.toRadians(Constants.cMA + cVTA)); //d = (h2-h1) / tan(a1+a2)
    SmartDashboard.putNumber("tanResult", Math.tan(Math.toRadians(Constants.cMA + cVTA)));
    SmartDashboard.putNumber("currentDistance", currentDistance);
    double distanceError = currentDistance - desiredDistance;
    SmartDashboard.putNumber("distanceError", distanceError);
    //double distanceErrorSum = 0;
    lastTimestamp = Timer.getFPGATimestamp();
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(distanceError) < iLimit) {
      distanceErrorSum += distanceError * dt;
    }
    double errorRate = (distanceError - lastError) / dt;

    //double distanceAdjust = kp_distancing * distanceError + ki * distanceErrorSum + kd * errorRate;
    lastError = distanceError;
    //double distanceAdjust = kp_distancing * distanceError + ki * distanceErrorSum;
    double distanceAdjust = kp_distancing * distanceError;
    leftSpeedTune += distanceAdjust;
    rightSpeedTune += distanceAdjust;
    SmartDashboard.putNumber("leftSpeedTune", leftSpeedTune);
    SmartDashboard.putNumber("rightSpeedTune", rightSpeedTune);
    //read data
  }

  public void autoPIDDistancing(double desiredDistance, double ty) //skipped no time to test sadly :/
  {
    //boolean runCamera = true;
    //if(runCamera){
      double cVTA = ty;//getYDegreeOffset(); //camera to vision target cneter y angle from cMA
      double currentDistance = (Constants.hOVT - Constants.hOL) / Math.tan(Math.toRadians(Constants.cMA + cVTA)); //d = (h2-h1) / tan(a1+a2)
      SmartDashboard.putNumber("tanResult", Math.tan(Math.toRadians(Constants.cMA + cVTA)));
      SmartDashboard.putNumber("currentDistance", currentDistance);
      double distanceError = (desiredDistance - currentDistance)/12.0;
      SmartDashboard.putNumber("distanceError", distanceError);
      Robot.kDrivetrain.driveP(0.23, distanceError);
      //runCamera = false;
    //}
  }


  public void autoLimelightDrive(double tx, double ty, double desiredDistance){

    double kp_angling = -0.008;
    //double minCommand = 0.01;
    double headingError = -tx;
    double steeringAdjust = 0.0;

    if(tx > 1) steeringAdjust = kp_angling*headingError;
    else if(tx < 1) steeringAdjust = kp_angling * headingError; 


    double cVTA = ty;//getYDegreeOffset(); //camera to vision target cneter y angle from cMA
    double currentDistance = (Constants.hOVT - Constants.hOL) / Math.tan(Math.toRadians(Constants.cMA + cVTA)); //d = (h2-h1) / tan(a1+a2)
    SmartDashboard.putNumber("tanResult", Math.tan(Math.toRadians(Constants.cMA + cVTA)));
    SmartDashboard.putNumber("currentDistance", currentDistance);
    double distanceError = (desiredDistance - currentDistance)/12.0;
    SmartDashboard.putNumber("distanceError", distanceError);
    Robot.kDrivetrain.drivePLimelight(0.23, distanceError, steeringAdjust);
  }

  public void resetSpeedTune()
  {
    leftSpeedTune = 0;
    rightSpeedTune = 0;
  }

  public double getLeftSpeedTune()
  {
    if(rightSpeedTune > Constants.speedLimelight) rightSpeedTune = Constants.speedLimelight;
    else if(rightSpeedTune < -Constants.speedLimelight) rightSpeedTune = -Constants.speedLimelight;
    return leftSpeedTune;

  }

  public double getRightSpeedTune()
  {
    if(leftSpeedTune > Constants.speedLimelight) leftSpeedTune = Constants.speedLimelight;
    else if(leftSpeedTune < -Constants.speedLimelight) leftSpeedTune = -Constants.speedLimelight;
    return rightSpeedTune;
  }

  //testing....autoangling wrosk



  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(null);
  }

}
