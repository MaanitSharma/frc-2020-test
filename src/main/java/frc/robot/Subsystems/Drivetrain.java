/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Robot;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.TestFalconCommand;
import frc.robot.OI;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import frc.robot.Subsystems.Limelight;
import edu.wpi.first.wpilibj.SPI;


/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  /*private static WPI_TalonSRX leftMotorFront = new WPI_TalonSRX(Constants.d_LeftTop);
  private static WPI_TalonSRX leftMotorRear = new WPI_TalonSRX(Constants.d_LeftBottom);
  private static WPI_TalonSRX rightMotorFront = new WPI_TalonSRX(Constants.d_RightTop);
  private static WPI_TalonSRX rightMotorRear = new WPI_TalonSRX(Constants.d_RightBottom);

  private static SpeedControllerGroup leftSide = new SpeedControllerGroup(leftMotorFront, leftMotorRear);
  private static SpeedControllerGroup rightSide = new SpeedControllerGroup(rightMotorFront, rightMotorRear);

  private static DifferentialDrive rDrive = new DifferentialDrive(leftSide, rightSide);
  */

  private static WPI_TalonSRX rightMaster;
  private static WPI_TalonSRX rightSlave;
  private static WPI_TalonSRX leftMaster;
  private static WPI_TalonSRX leftSlave;

  //private static WPI_TalonFX testFalconMaster;
  //private static WPI_TalonFX testFalconSlave;

  /*AHRS gyro = new AHRS(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  Pose2d pose;
  

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      getLeftEncoderVelocity() / 10.75 * 2 * Math.PI * Units.inchesToMeters(4.0) / 60,
      getRightEncoderVelocity() / 10.75 * 2 * Math.PI * Units.inchesToMeters(4.0) / 60);
  }

  

  @Override
  public void periodic(){
    pose = odometry.update(getHeading(), getLeftEncoderPosition(), getRightEncoderPosition());
  }

  */

  double setpoint = 0;
  final double iLimit = 1;
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;

  public Drivetrain(){
    initTalons();
    zeroEncoder();
    
  }

  private void initTalons(){
    rightMaster = new WPI_TalonSRX(Constants.d_RightTop);
    rightSlave = new WPI_TalonSRX(Constants.d_RightBottom);
    leftMaster = new WPI_TalonSRX(Constants.d_LeftTop);
    leftSlave = new WPI_TalonSRX(Constants.d_LeftBottom);
    

    //Init Falcons
    //testFalconMaster = new WPI_TalonFX(13);
    /*testFalconSlave = new WPI_TalonFX(12);

    testFalconMaster.setInverted(false);
    testFalconSlave.setInverted(true);

    testFalconSlave.follow(testFalconMaster);

    testFalconSlave.setInverted(InvertType.FollowMaster);

    */
    
    //Init Talons
    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);

    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);
    

    
    

    /*sensorPosition = getEncoders();
    error = setpoint - sensorPosition;
    autOutputSpeed = error * kP;
    */
  
  }

  /*public void driveRobot(){
    rDrive.tankDrive(Robot.oi.getPs4LeftYaxis(), Robot.oi.getPs4RightYaxis());
  }

  public void drive(double leftSpeed, double rightSpeed){
    rDrive.tankDrive(leftSpeed, rightSpeed);
  }
  */

  public void initPID(){
    Robot.kDrivetrain.zeroEncoder();
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();  
  }


  /*public void driveLeftSlave(){
    leftSlave.set(ControlMode.PercentOutput, 1);
  }

  public void driveRightSlave(){
    rightSlave.set(ControlMode.PercentOutput, 1);
  }

  public void driveLeftMaster(){
    leftMaster.set(ControlMode.PercentOutput, 1);

  public void driveRightMaster(){
    rightMaster.set(ControlMode.PercentOutput, 1);
  }
  */

  public void driveP(final double kP, final double setpoint){
    double sensorPosition = getEncodersDistance();
    double error = setpoint - sensorPosition;
    double outPutSpeed = kP * error;
    driveSlave(-outPutSpeed, -outPutSpeed);
    System.out.println("Speed: " + outPutSpeed);
    
    

  }

  public void drivePLimelight(final double kP, final double setpoint, double steeringAdjust) {
    double sensorPosition = getEncodersDistance();
    double error = setpoint - sensorPosition;
    double outPutSpeed = kP * error;
    //outPutSpeed += steeringAdjust;
    driveSlave(-outPutSpeed - steeringAdjust , -outPutSpeed + steeringAdjust);
    
  }



  public void drivePI(final double kP, final double kI, double setpoint){
    double sensorPosition = getEncodersDistance();
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(error) < iLimit) {
      errorSum += error * dt;
    }

    double outPutSpeed = kP * error + kI * errorSum;
    driveSlave(outPutSpeed, outPutSpeed);

    lastTimestamp = Timer.getFPGATimestamp();
  }

  public void drivePID(final double kP, final double kI, final double kD, double setpoint){
    double sensorPosition = getEncodersDistance();
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(error) < iLimit) {
      errorSum += error * dt;
    }

    double errorRate = (error - lastError) / dt;

    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;

    double outPutSpeed = kP * error + kI * errorSum + kD * errorRate;
    driveSlave(outPutSpeed, outPutSpeed);

  
  }



  public void configMagEncoder(WPI_TalonSRX talon, boolean phase) {
    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.cTalonCommTimeout);
    talon.setSensorPhase(phase);
  }

  public void driveSlave(double left, double right) {
    leftMaster.set(ControlMode.PercentOutput, -left);
    rightMaster.set(ControlMode.PercentOutput, -right);
  }

  public void driveJoystick(){
    driveSlave(Robot.oi.getPs4LeftYaxis(), Robot.oi.getPs4RightYaxis());
  }

  public void configEncodersForDrive(){
    configMagEncoder(leftMaster, true);
    configMagEncoder(rightMaster, true);
  }

  public double getEncodersDistance(){
    return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
  }


  public void zeroEncoder() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
  public void zeroSensor(){
    zeroEncoder();
  }

  public double getLeftEncoderPosition() {
    return leftMaster.getSelectedSensorPosition(0) * Constants.kDriveTicks2Feet;
  }

  public double getRightEncoderPosition() {
    return rightMaster.getSelectedSensorPosition(0) * Constants.kDriveTicks2Feet;
  }

  public double getRightEncoderVelocity() {
    return leftMaster.getSelectedSensorVelocity(0);
  }

  public double getLeftEncoderVelocity() {
    return rightMaster.getSelectedSensorVelocity(0);
  }

  /*public void runFalcons(){
    testFalconMaster.set(ControlMode.PercentOutput, 0.5);
  }
  */
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveCommand());


  }
}
