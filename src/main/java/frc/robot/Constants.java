package frc.robot;



public class Constants
{
    //General
    public static final int cTalonCommTimeout = 10; //ms

    //Drivetrain Ports
    public static int d_RightTop = 2;
    public static int d_RightBottom = 3;
    public static int d_LeftTop = 0;
    public static int d_LeftBottom = 1;

    //Shooter Ports
    public static int s_left = 14;
    public static int s_right = 15;

    //Intake Port
    public static int i_main = 5; //ctre cant retrieve...code is fine

    //Controller Ports
    public static int ps4_port = 0;

    public static int ps4LeftYaxis = 1;
    public static int ps4RightYaxis = 5;
    public static int ps4ShooterButton = 1;
    public static int ps4AutoAdjustButton = 2;
    public static int ps4ColorSensorButton = 3;
    public static int ps4IntakeOuttakeButton = 8;
    public static int ps4IntakeIntakeButton = 7;
    public static int ps4IntakeDeployButton = 5;

    //Pneumatics
    public static int pcmCanId = 20; //changed to 20
    public static int portOpen1 = 5; //forwardChannel1
    public static int portClose1 = 2; //reverseChannel1
    public static int portOpen2 = 7; //forwardChannel2
    public static int portClose2 = 0; //reverseChannel2

    //encoder
    public static final double kDriveTicks2Feet = 1.0 / 4096 * 4 * Math.PI / 12;
    public static final double kDeg2Talon4096Unit = 1 / 360.0 * 4096.0;
    public static final double kTalon4096Unit2Deg = 1 / kDeg2Talon4096Unit;

    //limelight calculation
    public static double cMA = 55; //camera mounting y angle from floor (facing this dir)
    public static double hOVT = 87.5; //vertical height of vision target center from floor, 6ft9.25in + 1ft5in //using table height currently
    public static double hOL = 14; //vertical height of Limelight from floor (inches)

    public static double speedLimelight = 0.3;

    

}   




