/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.networktables.NetworkTable; 
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance; 
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import java.util.Arrays;
import java.util.function.BiConsumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.Commands.AutoCommand;
import frc.robot.Commands.BoostedAutoCommand;
import frc.robot.Commands.ShootCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ColorSensor;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.PneumaticDeploy;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.Subsystems.Dingus;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If u change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization co
   */

   public static final Drivetrain kDrivetrain = new Drivetrain();
   public static final Shooter kShooter = new Shooter();
   //public static final Intake kIntake = new Intake();
   public static final PneumaticDeploy kIntakeDeploy = new PneumaticDeploy();

   public static OI oi = new OI();
   public static final ColorSensor cSensor = new ColorSensor();
   public static final Limelight limelight = new Limelight();
   public static final Dingus kDingus = new Dingus();


   public Command getAutnomousCommand(){
     TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
     config.setKinematics(Robot.kDrivetrain.getKinematics());
     
     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
       Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d()), new Pose2d(2.3, 1.2, new Rotation2d(90))),
       config
     );
    return null;

     /*RamseteCommand command = new RamseteCommand(
       trajectory, 
       kDrivetrain::getPose, 
       new RamseteController(2.0, 0.7), 
       kDrivetrain.getFeedForward(), 
       kDrivetrain.getKinematics(), 
       kDrivetrain.getSpeeds(), 
       kDrivetrain.getLeftPIDController(), 
       kDrivetrain.getRightPIDController(), 
       kDrivetrain::tankDriveVolts,
       kDrivetrain
       ); 
       return command.andThen(() -> kDrivetrain.tankDriveVolts(0,0));
      */
      
 
   }

   

  
   private Timer timer;

   //private final I2C.Port i2cPort = I2C.Port.kOnboard;
   //private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);


  @Override
  public void robotInit() {
   //timer = new Timer();

   //simple vision..enable to switch to usb cam
   /*UsbCamera viewCam = CameraServer.getInstance().startAutomaticCapture();
   viewCam.setResolution(640, 480);
   viewCam.setFPS(20);*/

  }
  @Override
  public void autonomousInit() {
    /*autoCommand = new AutoCommand();

    if (autoCommand != null){
      autoCommand.start();
    }
    */
    getAutnomousCommand();
    Robot.kDrivetrain.resetGyro();
    
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Left Drive Encoder Value", Robot.kDrivetrain.getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Drive Encoder Value", Robot.kDrivetrain.getRightEncoderPosition());



  }

  @Override
  public void teleopInit() {
    /*if (autoCommand != null){
      autoCommand.cancel();
    }
    Robot.kDrivetrain.resetGyro();
    */
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    SmartDashboard.putNumber("Left Drive Encoder Value", Robot.kDrivetrain.getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Drive Encoder Value", Robot.kDrivetrain.getRightEncoderPosition());
    Robot.kDrivetrain.printGyro();
    
    
  
    //Timer.delay(0.01);
  }

  @Override
  public void testInit() {

    
  }

  @Override
  public void testPeriodic() {

    

  }

}
