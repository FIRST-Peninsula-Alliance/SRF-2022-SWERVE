/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


//import edu.wpi.first.cscore.UsbCamera;
//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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

  //private static final String kDefaultAuto = "Default";
	//private static final String kCustomAuto = "My Auto";
	//private String m_autoSelected;
  private static final String defaultTeam = "Blue";
  private static final String customTeam = "Red";
  private String teamSelected;

  private SendableChooser<String> teamChooser = new SendableChooser<>();

  private static final String defaultAuto = "Auto1";
  private static final String customAuto = "Auto2";
  private static final String customAuto2 = "Auto3";
  private String AutoSelected;

  private SendableChooser<String> autoChooser = new SendableChooser<>();

  Joystick controller = new Joystick(0), Controller2 = new Joystick(1);

  //Button numbers for controls
  //Flap=right bumber
  //Shooter=right trigger
  //shooter shoot=right bumper
  //Pickup=left trigger
  //pickup pnumatic=left bumper
  //hood=y
  //slowmode=rightjoystickbutton
  //field orient=left options
  //recenter field orient=right options
  //pickup out=x button
  //climber up=dpad up
  //climber down=dpad down
  final int A = 1, B = 2, X = 3, Y = 4, leftBumper = 5, rightBumper = 6, leftOptionButton = 7, rightOptionButton = 8, leftJoystickButton = 9, rightJoystickButton = 10, dPadUp=11, dPadDown=12;

  //DriveBase Objects
  TalonFX frontLeftRot, frontRightRot, rearLeftRot, rearRightRot;
  TalonFX frontLeft, frontRight, rearLeft, rearRight;

  SRF_Swerve_Module FLModule;
  SRF_Swerve_Module FRModule;
  SRF_Swerve_Module RLModule;
  SRF_Swerve_Module RRModule;
  SRF_Swerve_Drive driveBase;


  //FIXME CLIMER INIT
  //Climbing motors
  //TalonSRX winch1, winch2, hookLift;
  
  //Ball Motors
  //Full rotation = 1024 Counts

  //TalonFX falcon;

  TalonSRX indexMotor;
  TalonSRX intakeMotor;
  CANSparkMax shooterMotor, backspinMotor;

  SparkMaxPIDController shooterPID,backspinPID;
  RelativeEncoder shooterEncoder,backspinEncoder;

  AHRS navx;
  // Block pixyBlock;
  // PixyCam pixy = new PixyCam();
  I2C wire = new I2C(Port.kOnboard, 0x0);
  ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  //Pneumatics
  DoubleSolenoid pickupSolenoid;
  DoubleSolenoid flapSolenoid;
  DoubleSolenoid hoodSolenoid;
  
  Compressor arnold;
  //Compressor arnold = new Compressor();

  boolean pickupDown = false; //if pickup is up, pickupUp is true
  boolean hoodDown = false;
  boolean flapDown = false;

  //counter to make pickup stay on for longer
  int pickupCounter;

  //PIDVALUES
  //drivekp was 0.55
  final double drivekP = 0.55, drivekI = 0, drivekD = 0;
  final double shootkP = 0.0005, shootkI = 0.00000027, shootkD = 0;
  final double backspinkP=0.00025, backspinkI=0.00000027, backspinkD=0;
//bruh
  //Joystick values for swerve drive
  double x, y, w;

  double zeroYaw, gyroAngle, gyroStartAngle,gyroTargetAngle,gyroRange;
  
  boolean testRotationController = true;
  
  //1 is blue, 2 is red
  int teamColor;
  int sensorProximity;
  //proximity switch is the value at which indexsensorvalue2 switched
  int ProximitySwitch;
  Boolean indexSensorValue2;


  //Index Sensor Booleans
  Boolean indexSensorValue1=false;  
  Boolean indexMotorToggle=false;
  Boolean indexShootToggle=false;
  Boolean indexTargetSwitch=false;

  double indexEncoderCounts;
  double indexDistanceBetween;
  double indexTargetCounts;
  
  //these are used to check if the joystick is giving out a command because they are no longer on a button but instead an axis
  Boolean leftTrigger=false, rightTrigger=false;

  //these are to check if you let go of the button
  Boolean letUpB = true, letUpX = true, letUpY=true,letUpA=true,letUpRBump=true,letUpLBump=true, letUpX2 = true, letUpPOV180 = true,letUpPOV0=true, letUpLeftTrigger = true, letUpRightTrigger = true,letUpLeftOptions=true, letUpRightOptions=true;
//let up X2 is the true x let up variable, used because for whatever reason letUpX is assinged to the "A" button.
//shooterspeedtemp is used to test different motor speeds and allow the speed of the motor to change without activating the shooter
  Double outtakeSpeed, shooterSpeed, shooterSpeedTemp,backspinSpeed,backspinSpeedTemp; 
  Boolean carouselVelPID = true;
  int prevCarouselPos;
  Boolean slowMode = true, unjam = false;
  int carouselStartPos;
  //0 -17 20 17
  final double offSetFL = 1.783, offSetFR = 1.854, offSetRL = 2.567, offSetRR = 1.361;

  //Tim's Room
  boolean timStart = false;
  Timer tim = new Timer();
  Timer indexTimer = new Timer();
  Timer match = new Timer();
  DigitalInput indexSensor1 = new DigitalInput(0);
  DigitalInput indexSensor2 = new DigitalInput(1);
  //UsbCamera cam;
  //FIXME FIELD ORIENT
  boolean fieldOriented = false;

  int dashboardDelay = 0;

  @Override
  public void robotInit() {
    
    //Team select
    teamChooser.addOption("Blue", defaultTeam);
		teamChooser.addOption("Red", customTeam);
		SmartDashboard.putData("Team Choices", teamChooser);
    
    //auto Select
    autoChooser.addOption("Auto1", defaultAuto);
    autoChooser.addOption("Auto2", customAuto);
    autoChooser.addOption("Auto3", customAuto2);
    SmartDashboard.putData("Auto Choices", autoChooser);

    //Driving initialization
    navx = new AHRS();

    frontLeft = new TalonFX(12);
    frontRight = new TalonFX(14);
    rearLeft = new TalonFX(18);
    rearRight = new TalonFX(16);

    frontLeftRot = new TalonFX(11);
    frontRightRot = new TalonFX(13);
    rearLeftRot = new TalonFX(17);
    rearRightRot = new TalonFX(15);

    

    FLModule = new SRF_Swerve_Module(0,11, 12, drivekP, drivekI, drivekD, offSetFL);
    FRModule = new SRF_Swerve_Module(1,13, 14, drivekP, drivekI, drivekD, offSetFR);
    RLModule = new SRF_Swerve_Module(2,17, 18, drivekP, drivekI, drivekD, offSetRL);
    RRModule = new SRF_Swerve_Module(3,15, 16, drivekP, drivekI, drivekD, offSetRR);
    

    driveBase = new SRF_Swerve_Drive(FLModule, FRModule, RLModule, RRModule, 25.0, 21.0, 32.65);

    //climbing initialization
    // winch1 = new TalonSRX(14);
    // winch2 = new TalonSRX(15);
    // hookLift = new TalonSRX(13);
    
    //Ball Manipulating initialization
    

    intakeMotor = new TalonSRX(7);
    indexMotor = new TalonSRX(8);
    indexMotor.configNominalOutputForward(0, 0);
		indexMotor.configNominalOutputReverse(0, 0);
		indexMotor.configPeakOutputForward(0.5, 0);
		indexMotor.configPeakOutputReverse(-0.5, 0);
    indexMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    //was 0.35

    indexMotor.config_kP(0, 0.35, 0);
    indexMotor.config_kI(0, 0, 0);
    indexMotor.config_kD(0, 0, 0);
    



    shooterMotor = new CANSparkMax(9, MotorType.kBrushless);
    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterPID = shooterMotor.getPIDController();
    shooterPID.setP(shootkP);
    shooterPID.setI(shootkI);
    shooterPID.setD(shootkD);
    shooterPID.setIMaxAccum(1.0, 0);
    shooterEncoder = shooterMotor.getEncoder();
    

    backspinMotor = new CANSparkMax(10, MotorType.kBrushless);
    backspinMotor.setIdleMode(IdleMode.kCoast);
    backspinPID = backspinMotor.getPIDController();
    backspinPID.setP(backspinkP);
    backspinPID.setI(backspinkI);
    backspinPID.setD(backspinkD);
    backspinPID.setIMaxAccum(1.0, 0);
    backspinEncoder = backspinMotor.getEncoder();
    
    //FIXME First line only
    //arnold = new Compressor(0,PneumaticsModuleType.REVPH);
    //arnold.setClosedLoopControl(true);
    
    
    //pickupSolenoid= new DoubleSolenoid(9,PneumaticsModuleType.REVPH,3,4);
    //hoodSolenoid = new DoubleSolenoid(9,PneumaticsModuleType.REVPH,1,6);
    //FIXME stuff for flap
    //flapSolenoid = new DoubleSolenoid(0,0,0);
    
    //camera
    //cam = CameraServer.startAutomaticCapture();
    //cam.setResolution(320, 240);

    

  }
 
  public void disabledPeriodic(){
    //SmartDashboard.putNumber("HookLift", hookLift.getSelectedSensorPosition());
    /*SmartDashboard.putNumber("Front Left Sensor", frontLeftRot.getSelectedSensorPosition());
    SmartDashboard.putNumber("Front Right Sensor", frontRightRot.getSelectedSensorPosition());
    SmartDashboard.putNumber("Rear Left Sensor", rearLeftRot.getSelectedSensorPosition());
    SmartDashboard.putNumber("Rear Right Sensor", rearRightRot.getSelectedSensorPosition());*/
    /*SmartDashboard.putNumber("gyro", navx.getAngle()%360);*/
  }

  @Override
  public void autonomousInit() {
     timStart = false;
     tim.stop();
     tim.reset();
     AutoSelected = autoChooser.getSelected();
     
     //hoodSolenoid.set(Value.kForward);
     shooterMotor.set(0);
     backspinMotor.set(0);
     
     match.start();
     
     gyroStartAngle=navx.getAngle();
     zeroYaw = navx.getAngle() % 360;
     if(zeroYaw < 0){
       zeroYaw += 360;
     }

     gyroStartAngle=navx.getAngle();

    }
    
   

   @Override
   public void autonomousPeriodic() {
    SmartDashboard.putNumber("gyro angle", (navx.getAngle()));
    SmartDashboard.putNumber("gyro start angle", gyroStartAngle);
    SmartDashboard.putNumber("gyro target Angle", gyroTargetAngle);
    SmartDashboard.putNumber("time", tim.get());
    gyroRange=10;
    gyroAngle=navx.getAngle();
    
    if(AutoSelected=="Auto1"){
      SmartDashboard.putNumber("auto", 1);
    }else if(AutoSelected=="Auto2"){
      SmartDashboard.putNumber("auto", 2);
    }else if(AutoSelected=="Auto3"){
      SmartDashboard.putNumber("auto", 3);
    }

     //the first line checks to see if we have to cross over 0
    //the second line is checking to see if the gyroangle is between 135 degrees to the right and 145 degrees to the right
    //if it isnt it rotates the robot clockwise until it is inbetween
    // if(Math.abs(gyroStartAngle+180)<140){
    //   if(Math.abs(gyroAngle+180)<((360-(Math.abs(Math.abs(gyroStartAngle+180)-140))))&Math.abs(gyroAngle+180)>((360-(Math.abs(Math.abs(gyroStartAngle+180)-140)))-10)){
    //     driveBase.set(0,0,0);
      
    //   }else{
    //     driveBase.set(0,0,0.05);
        
    //   }
    // }else
    //   if(Math.abs(gyroAngle+180)<(Math.abs(gyroAngle+180)-140)&Math.abs(gyroAngle+180)>(Math.abs(gyroAngle+180)-10-140)){
    //     driveBase.set(0,0,0);
    //   }else{
    //     driveBase.set(0,0,0.05);
    //   } 
    //timer based system
    // if(timStart == false) {
    //       tim.start();
    //       timStart = true;
    // }
    // if(tim.get()<4){
    //   driveBase.set(0, -0.25, 0);
    // }
    // if(tim.get()<7&tim.get()>4){
    //     gyroTargetAngle=gyroStartAngle+140;
    //   if(gyroAngle>(gyroTargetAngle-gyroRange)&gyroAngle<(gyroTargetAngle+gyroRange)){
    //     driveBase.set(0, 0, 0);
    //   }else{
    //     driveBase.set(0,0,0.15);
    //   }
    // }
    // if(tim.get()>7){
    //   driveBase.set(0,0,0);
    // }
  
  }

  @Override
  public void teleopInit() {
    letUpRBump = true;
    timStart = false;
    
    //pickupSolenoid.set(Value.kForward);
    //hoodSolenoid.set(Value.kReverse);
    shooterSpeedTemp=4600.0;
    backspinSpeedTemp=2300.0;
    indexTargetCounts=indexMotor.getSelectedSensorPosition();
    indexShootToggle=false;
    pickupCounter=100;
    teamSelected = teamChooser.getSelected();
    if(teamSelected=="Blue"){
      teamColor=1;
    }else{
      teamColor=2;
    }

  }

  @Override
  public void teleopPeriodic() {
    teamSelected = teamChooser.getSelected();
    shooterSpeed = 0.0;
    outtakeSpeed = 0.0;
    backspinSpeed = 0.0;
    indexSensorValue1 = indexSensor1.get();
    indexSensorValue2 = indexSensor2.get();
    sensorProximity = colorSensor.getProximity();
    
    SmartDashboard.putNumber("frontleftrot", frontLeftRot.getSelectedSensorPosition());
    SmartDashboard.putNumber("FRtrot",  frontRightRot.getSelectedSensorPosition());
    SmartDashboard.putNumber("BLrot", rearLeftRot.getSelectedSensorPosition());
    SmartDashboard.putNumber("BR", rearRightRot.getSelectedSensorPosition()%26214);
    SmartDashboard.putNumber("IndexMotorEncoder", indexMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("IndexTargetCounts", indexTargetCounts);
    SmartDashboard.putBoolean("indexTargetSwitch", indexTargetSwitch);
    SmartDashboard.putBoolean("indexSensorValue2", indexSensorValue2);
      SmartDashboard.putBoolean("indexSensorValue1", indexSensorValue1);
      SmartDashboard.putBoolean("IndexTargetSwitch", indexTargetSwitch);


    x = controller.getRawAxis(0);
    y = controller.getRawAxis(1);
    w = controller.getRawAxis(4);
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
    

    //FIXME SLOW MODE
    if(slowMode) {
      x *= .25;
      y *= .25;
    }

    
    
    //w * 0.7 limits rotational speed
    if(fieldOriented)
      driveBase.set(x, y*-1, w*.7, gyroAngle);
    else
      driveBase.set(x, y*-1, w*.7);

    //this gives the triggers boolean outputs, its at 0.05 so it removes noise
    if((Math.abs(controller.getRawAxis(2))>0.05)){
      leftTrigger=true;
    }else{
      leftTrigger=false;
    }
    SmartDashboard.putBoolean("leftTrigger", leftTrigger);
    
    if((Math.abs(controller.getRawAxis(3))>0.05)){
      rightTrigger=true;
    }else{
      rightTrigger=false;
    }
    SmartDashboard.putBoolean("rightTrigger", rightTrigger);
    

    //color sensor and prox code
    Color detectedColor = colorSensor.getColor();
    double IR = colorSensor.getIR();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    int proximity = colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);

    //this is just switching the proximity value into a boolean for easy use
    if(sensorProximity>ProximitySwitch){
      indexSensorValue2=true;
    }else{
      indexSensorValue2=false;
    }
    //3826 is a place holder switch this value
    //1 is blue, 2 is red
    if(indexSensorValue2==true){
      if(teamColor==1){
        if(detectedColor.red>0.32){
          //activate motor at low velocity also do the index
        }
      }else if(teamColor==2){
        if(detectedColor.blue>0.3){
          //activate motor at low velocity also do the index
        }
      }
    }

    if(teamSelected=="Blue"){
      teamColor=1;
    }else{
      teamColor=2;
    }





    //controller code starts here


    //pickup motor code
    if((leftTrigger==true) && letUpLeftTrigger) {
      intakeMotor.set(ControlMode.PercentOutput, 1);
      pickupCounter=0;
      letUpLeftTrigger = false;
      } else if(leftTrigger==false && !letUpLeftTrigger) {
        letUpLeftTrigger = true;
        if(pickupCounter<25){
          intakeMotor.set(ControlMode.PercentOutput, 1);
          pickupCounter++;
        }else{
          intakeMotor.set(ControlMode.PercentOutput, 1);
        }   
      }else if((controller.getRawButton(B))){
        intakeMotor.set(ControlMode.PercentOutput, -1);
      }else{
        intakeMotor.set(ControlMode.PercentOutput, 0);
      }
      if(leftTrigger==false &&pickupCounter<100){
        pickupCounter++;
      }
      if(pickupCounter<25){
        intakeMotor.set(ControlMode.PercentOutput, 1);
      }
      SmartDashboard.putNumber("pickupcounter", pickupCounter);
      
      

      //resetting field orient
      if(controller.getRawButton(rightOptionButton) && letUpRightOptions) {
          zeroYaw = navx.getAngle() % 360;
          if(zeroYaw < 0)
            zeroYaw += 360;
          letUpRightOptions = false;
        } else if(!controller.getRawButton(rightOptionButton) && !letUpRightOptions) {
          letUpRightOptions = true;
        }
        
      //actuation code for pickup
      if(controller.getRawButtonPressed(leftBumper) && letUpLBump)
      {
        if(pickupDown == false) {
          pickupSolenoid.set(Value.kReverse);
          pickupDown = true;
        } else {
          pickupSolenoid.set(Value.kForward);
          pickupDown = false;
        }
        letUpLBump = false;
      } else if(!controller.getRawButton(leftBumper)) {
        letUpLBump = true;
      }  
  
      //Slow Mode
      if(controller.getRawButton(rightJoystickButton)){
        slowMode=true; 
      }else if(!controller.getRawButton(rightJoystickButton)){
        slowMode=false;
      }
  
      //index sensors
      //switch indexsensorvalue2 to true once you have sensor
      if(indexSensorValue2==true){
        if(indexSensorValue1==false){
          if(indexTargetSwitch==false){
            indexTargetSwitch=true;
            indexTargetCounts=indexMotor.getSelectedSensorPosition()+40960;
        }
      }
    } 
      
      
        
    if(Math.abs(Math.abs(indexTargetCounts)-Math.abs(indexMotor.getSelectedSensorPosition()))>300){
      indexMotor.set(ControlMode.Position, indexTargetCounts);
    }else{
      indexTargetSwitch=false;
     
    } 
    SmartDashboard.putNumber("indexdifference", Math.abs(Math.abs(indexTargetCounts)-Math.abs(indexMotor.getSelectedSensorPosition())));
    //Hood
    if(controller.getRawButtonPressed(Y) && letUpY)
    {
      
      if(hoodDown == false) {
        hoodSolenoid.set(Value.kReverse);
        hoodDown = true;
      } else {
        hoodSolenoid.set(Value.kForward);
        hoodDown = false;
      }
      letUpY = false;
    } else if(!controller.getRawButton(Y)) {
      letUpY = true;
    }

    //field orient code
    if(controller.getRawButton(leftOptionButton) && letUpLeftOptions) {
      fieldOriented = !fieldOriented;
      letUpPOV180 = false;
    } else if(!controller.getRawButton(leftOptionButton) &&!letUpLeftOptions) {
      letUpPOV180 = true;
    }
  
  

     //FIXME FLAP CODE, SWITCH OFF RIGHT BUMPER
     //flap code
    //  if(controller.getRawButtonPressed(rightBumper) && letUpRBump)
    // {
    //   if(flapDown == false) {
    //     flapSolenoid.set(Value.kReverse);
    //     flapDown = true;
    //   } else {
    //     flapSolenoid.set(Value.kForward);
    //     flapDown = false;
    //   }
    //   letUpRBump = false;
    // } else if(!controller.getRawButton(rightBumper)) {
    //   letUpRBump = true;
    // }

    //brings up flap if above 25% speed
    // if((Math.abs(Y)>0.25||Math.abs(X)>0.25)&&flapDown==true){
    //   flapSolenoid.set(Value.kForward);
    //   flapDown=false;
    // }
    
    
    
    //Shooter function
    if(controller.getRawButton(rightBumper) && shooterEncoder.getVelocity() > 2800&&backspinEncoder.getVelocity()>1400){
      //the if statment makes sure it only does this once until the next ball shoots
      if(indexShootToggle==false){
        indexShootToggle=true;
        indexTargetCounts+=10240;
      }else{
        indexShootToggle=false;
      }
      //1024 is the amount of counts it needs to mode, not tuned
    }   
    
    //shooter warmup
    if(rightTrigger){
      shooterSpeed=-3050.0;
      backspinSpeed=1500.0;
      
      //SmartDashboard.putNumber("ShooterSpeed",shooterSpeedTemp);
      letUpRightTrigger=false;
    }else if(!rightTrigger){
      letUpRightTrigger=true;;
    }
    
    //FIXME for comp code
    //if(Timer.getMatchTime() <= 30){
      SmartDashboard.putNumber("velocityofbackspin", backspinEncoder.getVelocity());
      //climber up
      if(controller.getPOV()==0||controller.getPOV()==315||controller.getPOV()==45) {
            letUpPOV0 = false;
          } else if(controller.getPOV()!=0&&controller.getPOV()!=315&&controller.getPOV()!=45 && !letUpPOV0) {
            letUpPOV0 = true;
          }

      //climberDown
      if(controller.getPOV()==180||controller.getPOV()==225||controller.getPOV()==135) {
        letUpPOV0 = false;
      } else if(controller.getPOV()!=180&&controller.getPOV()!=225&&controller.getPOV()!=135 && !letUpPOV0) {
        letUpPOV0 = true;
      }
  //}


    //example
    // if(controller.getRawButton(leftBumper) && letUpBack) {
    //   falcon.set(ControlMode.PercentOutput, 0.1);
    //     letUpBack = false;
    //   } else if(!controller.getRawButton(leftBumper) && !letUpBack) {
    //     letUpBack = true;
    //     falcon.set(ControlMode.PercentOutput, 0.0);
    //   }

    if(controller.getRawButton(A) && letUpA) {
      indexTargetCounts-=124000;
        letUpA = false;
      } else if(!controller.getRawButton(A) && !letUpA) {
        letUpA = true;
      }
      SmartDashboard.putBoolean("letupA", letUpA);
     
      
   
    //FIXME Climber CODE
    //85000 at low power(.1 or .2)
    // if(hookLift.getSelectedSensorPosition() > -865000 && controller.getRawButton(Y)) { //Y = up
    //   if(hookLift.getSelectedSensorPosition() < -835000)
    //     hookLift.set(ControlMode.PercentOutput, -0.2);
    //   else
    //     hookLift.set(ControlMode.PercentOutput, -0.6);
    // //0 at low power(.1)
    // }else if(hookLift.getSelectedSensorPosition() < -5000 && controller.getRawButton(X)) {
    //   if(hookLift.getSelectedSensorPosition() > -50000)
    //     hookLift.set(ControlMode.PercentOutput, 0.2);
    //   else
    //     hookLift.set(ControlMode.PercentOutput, 0.6);
    // } else
    //   hookLift.set(ControlMode.PercentOutput, 0);

    // //slows down robot when the hook is up
    // if(hookLift.getSelectedSensorPosition() < -100000) {
    //   slowMode = true;
    // }

    // if(controller.getPOV() == 0 /*&& Timer.getMatchTime() <= 30*/) {
    //   winch1.set(ControlMode.PercentOutput, 1);
    //   winch2.set(ControlMode.Follower, 14);
    // } else {
    //   winch1.set(ControlMode.PercentOutput, 0);
    //   winch2.set(ControlMode.Follower, 14);
    // }

    
    
    SmartDashboard.putNumber("POV", controller.getPOV());
    //TIMER CODE
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());


    
    

    if(shooterSpeed == 0){
      shooterMotor.set(0);
    }else{
      shooterPID.setReference(shooterSpeed, ControlType.kVelocity);
    }

    if(backspinSpeed==0){
      backspinMotor.set(0);
    }else{
      backspinPID.setReference(backspinSpeed, ControlType.kVelocity);
    }

    if(match.get() > 110) {
      arnold.disable();;
    }

    //SmartDashboard.putNumber("ShooterSpeed",shooterSpeedTemp);

    //SmartDashboard commands
    if(dashboardDelay == 3) {
      // SmartDashboard.putBoolean("Hood Down", hoodDown);
      // SmartDashboard.putBoolean("Field Oriented", fieldOriented);
      // SmartDashboard.putBoolean("Shoot", shooterEncoder.getVelocity() > 4000);
      
      //SmartDashboard.putNumber("distance to base", colorSensor.getProximity());
      //SmartDashboard.putBoolean("distance to base", colorSensor.getProximity() < 3);

      //SmartDashboard.putNumber("I Accumulation", shooterPID.getIAccum());
      //SmartDashboard.putNumber("color wheel motor controller", spinnerMotor.getSelectedSensorPosition());
      /*SmartDashboard.putNumber("SpinnerCounts", spinnerMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("POV", controller.getPOV());
      SmartDashboard.putBoolean("SpinnerDown", spinnerDown);*/
      /*SmartDashboard.putNumber("CarouselP", caroPoskP);
      SmartDashboard.putNumber("CarouselI", caroPoskI);
      SmartDashboard.putNumber("CarouselD", caroPoskD);
      SmartDashboard.putNumber("CarouselSpeed", carouselSpeed);
      SmartDashboard.putNumber("Carousel I Accum", carousel.getIntegralAccumulator());
      SmartDashboard.putBoolean("carouselVelPID", carouselVelPID);
      
      // SmartDashboard.putNumber("velocity", shooterEncoder.getVelocity());
      //SmartDashboard.putNumber("ShooterSpeed",shooterSpeedTemp);
      // SmartDashboard.putNumber("Level",lemon);/*
      // SmartDashboard.putNumber("carousel Pos", carousel.getSelectedSensorPosition());
      // SmartDashboard.putNumber("carousel Vel", carousel.getSelectedSensorVelocity());
      //SmartDashboard.putNumber("HookLift", hookLift.getSelectedSensorPosition());
      //driveBase.displaySmartDashboard(true, true, true, true);
      /*SmartDashboard.putNumber("Zero Yaw", zeroYaw);
      SmartDashboard.putNumber("Gyro", navx.getAngle());*/
      /*SmartDashboard.putNumber("X", x);
      SmartDashboard.putNumber("Y", y);
      SmartDashboard.putNumber("W", w);*/
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