/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  Joystick controller = new Joystick(0), Controller2 = new Joystick(1);

  //Button numbers for controls
  final int A = 2, B = 3, X = 1, Y = 4, leftBumper = 5, rightBumper = 6, leftTrigger = 7, rightTrigger = 8, back = 9, start = 10, dPadUp=11, dPadDown=12;


  //AutoTicker that goes through each path
  int AutoTicker=0;
  //DriveBase Objects
  //these need to be updated for the 2022 robot
  //TalonSRX frontLeftRot, frontRightRot, rearLeftRot, rearRightRot;
  //CANSparkMax frontLeft, frontRight, rearLeft, rearRight;

  SRF_Swerve_Module FLModule;
  SRF_Swerve_Module FRModule;
  SRF_Swerve_Module RLModule;
  SRF_Swerve_Module RRModule;
  SRF_Swerve_Drive driveBase;

  //Navx gyro Board
  AHRS navx;

  //unsure of what this is
  I2C wire = new I2C(Port.kOnboard, 0x0);

  

  

  final double drivekP = 16, drivekI = 0.00013, drivekD = 0;

  //Joystick values for swerve drive
  double x, y, w;

 
  
  int TestCounter=0;
  
  double zeroYaw, gyroAngle, gyroStartAngle;
  
  boolean testRotationController = true;

  Boolean letUpB = true, letUpX = true, letUpY=true, letUpStart = true, letUpBack = true, letUpRBump, letUpX2 = true, letUpPOV180 = true;
//let up X2 is the true x let up variable, used because for whatever reason letUpX is assinged to the "A" button.
//shooterspeedtemp is used to test different motor speeds and allow the speed of the motor to change without activating the shooter
  
  Boolean slowMode = true;
  //0 -17 20 17
  final int offSetFL = 0, offSetFR = 0, offSetRL = 0, offSetRR = 0;

  //Timer
  //timer is named tim
  boolean timStart = false;
  Timer tim = new Timer();

  Timer match = new Timer();



  boolean fieldOriented = true;

  int dashboardDelay = 0;

  @Override
  public void robotInit() {
    //Driving initialization
    navx = new AHRS();


    //needs to be updated for 2022
    frontLeft = new CANSparkMax(2, MotorType.kBrushless);
    frontRight = new CANSparkMax(8, MotorType.kBrushless);
    rearLeft = new CANSparkMax(4, MotorType.kBrushless);
    rearRight = new CANSparkMax(6, MotorType.kBrushless);

    //needs to be updated for 2022
    frontLeftRot = new TalonSRX(1);
    frontRightRot = new TalonSRX(7);
    rearLeftRot = new TalonSRX(3);
    rearRightRot = new TalonSRX(5);

    FLModule = new SRF_Swerve_Module(1, 2, drivekP, drivekI, drivekD, offSetFL);
    FRModule = new SRF_Swerve_Module(7, 8, drivekP, drivekI, drivekD, offSetFR);
    RLModule = new SRF_Swerve_Module(3, 4, drivekP, drivekI, drivekD, offSetRL);
    RRModule = new SRF_Swerve_Module(5, 6, drivekP, drivekI, drivekD, offSetRR);

    driveBase = new SRF_Swerve_Drive(FLModule, FRModule, RLModule, RRModule, 25.0, 21.0, 32.65);

  }
 
  public void disabledPeriodic(){
    
  }

  @Override
  public void autonomousInit() {
     timStart = false;
     tim.stop();
     tim.reset();

     match.start();
    
     zeroYaw = navx.getAngle() % 360;
     if(zeroYaw < 0){
       zeroYaw += 360;
     }
        
      gyroStartAngle=navx.getAngle();
    }
   
   

   @Override
   public void autonomousPeriodic() {

    
  }

  @Override
  public void teleopInit() {
    letUpRBump = true;
    timStart = false;
  }

  @Override
  public void teleopPeriodic() {
    
    x = controller.getRawAxis(0);
    y = controller.getRawAxis(1);
    w = controller.getRawAxis(2);
    gyroAngle = navx.getAngle() - zeroYaw;
    gyroAngle %= 360;

    if(gyroAngle < 0) 
      gyroAngle += 360;
    //Sets a deadband of .03 and adjusts the controller values so they don't jump from 0 to .3 and
    //instead are out of the remaining .97 range
    if(Math.abs(x) < .03)
      x = 0.0;
    else if(x > 0)
      x = (x - .05)/.97;
    else
      x = (x + .05)/.97;

    if(Math.abs(y) < .03)
      y = 0.0;
    else if(y > 0)
      y = (y - .05)/.97;
    else
      y = (y + .05)/.97;
    
    if(Math.abs(w) < 0.01)
      w = 0.0;
    else if(w > 0)
      w = (w - .05)/.99;
    else
      w = (w + .05)/.99;
    
    //Squares the values to slow down the speeds closer to the center
    x *= Math.abs(x);
    y *= Math.abs(y);
    w *= Math.abs(w);

    if(slowMode) {
      x *= .5;
      y *= .5;
    }
   

    //w * 0.7 limits rotational speed
    if(fieldOriented)
      driveBase.set(x, y*-1, w*.7, gyroAngle);
    else
      driveBase.set(x, y*-1, w*.7);

    if(controller.getRawButton(back) && letUpBack) {
      zeroYaw = navx.getAngle() % 360;
      if(zeroYaw < 0)
        zeroYaw += 360;
      letUpBack = false;
    } else if(!controller.getRawButton(back) && !letUpBack) {
      letUpBack = true;
    }
    

      if(controller.getRawButton(Y)){
        slowMode=true;
        
      }else if(!controller.getRawButton(Y)){
        slowMode=false;
      }


    
     
    
   

    if(controller.getPOV() == 180 && letUpPOV180) {
      fieldOriented = !fieldOriented;
      letUpPOV180 = false;
    } else if(controller.getPOV() != 180) {
      letUpPOV180 = true;
    }
       

    //SmartDashboard commands
    if(dashboardDelay == 3) {
      
      dashboardDelay = 0;
    }
    dashboardDelay++;
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    
    
  }
}