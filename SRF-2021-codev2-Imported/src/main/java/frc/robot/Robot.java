/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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

  private static final String defaultTeam = "Blue";
  private static final String customTeam = "Red";
  private static final String customTeam2 = "Off"; 
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


  
  //Ball Motors
  //Full rotation = 1024 Counts
  TalonSRX agitatorMotor;
  TalonSRX indexMotor;
  TalonSRX intakeMotor;
  TalonFX shooterMotor, backspinMotor;

  
  // Block pixyBlock;
  // PixyCam pixy = new PixyCam();
  I2C wire = new I2C(Port.kOnboard, 0x0);
  ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  AHRS navx = new AHRS(SPI.Port.kOnboardCS0);


  //Pneumatics
  
  
   //Compressor arnold = new Compressor(1,PneumaticsModuleType.REVPH);
   
  DoubleSolenoid pickupSolenoid= new DoubleSolenoid(1,PneumaticsModuleType.REVPH,0,4);
  DoubleSolenoid hoodSolenoid = new DoubleSolenoid(1,PneumaticsModuleType.REVPH,2,6);
  DoubleSolenoid flapSolenoid = new DoubleSolenoid(1,PneumaticsModuleType.REVPH,1,5);
  DoubleSolenoid climberPinsSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 7, 3);
  DoubleSolenoid bigClimberSolenoid = new DoubleSolenoid(1,PneumaticsModuleType.REVPH,8,9);

  boolean pickupDown = false; //if pickup is up, pickupUp is true
  boolean hoodDown = false;
  boolean flapDown = false;
  boolean climberPinsout = false;
  boolean bigclimberdown = false;

  //counter to make pickup stay on for longer
  int pickupCounter;

  //PIDVALUES
  //drivekp was 0.55
  final double drivekP = 0.55, drivekI = 0, drivekD = 0;
  //final double shootkP = 0.0005, shootkI = 0.00000027, shootkD = 0;
  //final double backspinkP=0.00025, backspinkI=0.00000027, backspinkD=0;

  
  //Joystick values for swerve drive
  double x, y, w;

  double zeroYaw, gyroAngle, gyroStartAngle,gyroTargetAngle,gyroRange;
    
  //1 is blue, 2 is red
  int teamColor;
  int sensorProximity;
  //proximity switch is the value at which indexsensorvalue2 switched
  int ProximitySwitch=100;
  Boolean indexSensor2Switch=true;
  Boolean indexAllow=true;
  //Index Sensor Booleans
  Boolean indexSensorValue2=false;
  Boolean indexSensorValue1=false;  
  Boolean indexShootToggle=false;
  Boolean indexTargetSwitch=false;
  double indexTargetCounts;
  double indexCountsDifference;
  

  
  boolean agitatorCurrentSwitch=false;
  boolean agitatorTimerSwitch=false;



  Boolean frontLeftSwitch=false;
  //these are used to check if the joystick is giving out a command because they are no longer on a button but instead an axis
  //this is for xbox controller not logitech, xbox controller sees triggers as an axis
  Boolean leftTrigger=false, rightTrigger=false;

  //these are to check if you let go of the button
  //C2 is for controller 2
  Boolean letUpPOV180C2 = true,letUpPOV0C2=true, letUpPOV90C2=true,letUPPOV270C2=true;
  Boolean letUpB = true, letUpX = true, letUpY=true,letUpA=true,letUpRBump=true,letUpLBump=true, letUpX2 = true, letUpPOV180 = true,letUpPOV0=true, letUpLeftTrigger = true, letUpRightTrigger = true,letUpLeftOptions=true, letUpRightOptions=true, letUpRightJoystickButton, letUpLeftJoystickButton;
//let up X2 is the true x let up variable, used because for whatever reason letUpX is assinged to the "A" button.
//shooterspeedtemp is used to test different motor speeds and allow the speed of the motor to change without activating the shooter
  double shootertargetVelocity_UnitsPer100ms;
  double backSpintargetVelocity_UnitsPer100ms;  
  double shooterSpeedTemp=0;
  double shooterSpeed, backspinSpeed;
  Boolean slowModeToggle;
  Boolean slowMode = false;
  final double offSetFL = 1.783, offSetFR = 1.854, offSetRL = 2.567, offSetRR = 1.361;

  
  
  double autoFrontLeft, autoFrontRight, autoBackLeft,autoBackRight;
  double autoStartAverage;
  double autoAverage;

  int AutonomousChosen;

  //Tim's Room
  boolean timStart = false;
  Timer tim = new Timer();
  Timer indexTimer = new Timer();
  Timer match = new Timer();
  Timer AgitatorTimer = new Timer();
  DigitalInput indexSensor1 = new DigitalInput(0);
  
  UsbCamera cam;
  //FIXME FIELD ORIENT
  boolean fieldOriented = true;

  int dashboardDelay = 0;

  @Override
  public void robotInit() {
    
    //Compressor arnold = new Compressor(1,PneumaticsModuleType.REVPH);
    

    //Team select
    teamChooser.addOption("Blue", defaultTeam);
		teamChooser.addOption("Red", customTeam);
    teamChooser.addOption("Off", customTeam2);
		SmartDashboard.putData("Team Choices", teamChooser);
    
    //auto Select
    autoChooser.addOption("Auto1", defaultAuto);
    autoChooser.addOption("Auto2", customAuto);
    autoChooser.addOption("Auto3", customAuto2);
    SmartDashboard.putData("Auto Choices", autoChooser);

    //Driving initialization

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
    
    agitatorMotor = new TalonSRX(6);
    intakeMotor = new TalonSRX(7);
    intakeMotor.setNeutralMode(NeutralMode.Coast);
    indexMotor = new TalonSRX(8);
    indexMotor.setNeutralMode(NeutralMode.Brake);
    indexMotor.configNominalOutputForward(0, 0);
		indexMotor.configNominalOutputReverse(0, 0);
		indexMotor.configPeakOutputForward(0.3, 0);
		indexMotor.configPeakOutputReverse(-0.3, 0);
    indexMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    //was 0.35
    indexMotor.config_kP(0, 0.5, 0);
    indexMotor.config_kI(0, 0, 0);
    indexMotor.config_kD(0, 0, 0);
    
    shooterMotor = new TalonFX(20);
    shooterMotor.configNominalOutputForward(0, 0);
	  shooterMotor.configNominalOutputReverse(0, 0);
    shooterMotor.configPeakOutputForward(1, 0);
		shooterMotor.configPeakOutputReverse(-1, 0);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0, 0);
    //was 0.05
    //was 0.00000027
    shooterMotor.config_kP(0,0.07,0);
    shooterMotor.config_kI(0, 0.00000027, 0);
    shooterMotor.config_kD(0, 0, 0);
    shooterMotor.config_kF(0, 0, 0);
    shooterMotor.overrideLimitSwitchesEnable(false);
    shooterMotor.overrideSoftLimitsEnable(false);
    shooterMotor.enableVoltageCompensation(false);
    shooterMotor.enableVoltageCompensation(true);
    shooterMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 0.5),0);
    

    backspinMotor = new TalonFX(21);
    backspinMotor.configNominalOutputForward(0, 0);
		backspinMotor.configNominalOutputReverse(0, 0);
    backspinMotor.configPeakOutputForward(1, 0);
	  backspinMotor.configPeakOutputReverse(-1, 0);
    backspinMotor.setNeutralMode(NeutralMode.Coast);
    backspinMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0, 0);
    backspinMotor.config_kP(0,0.05,0);
    backspinMotor.config_kI(0, 0.00000027, 0);
    backspinMotor.config_kD(0, 0, 0);
    backspinMotor.overrideLimitSwitchesEnable(false);
    backspinMotor.overrideSoftLimitsEnable(false);
    backspinMotor.enableVoltageCompensation(false);
    backspinMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 0.5),0);
    
    pickupSolenoid.set(Value.kReverse);
    hoodSolenoid.set(Value.kReverse);
    flapSolenoid.set(Value.kReverse);
    climberPinsSolenoid.set(Value.kForward);
    bigClimberSolenoid.set(Value.kForward);


    //camera

    cam = CameraServer.startAutomaticCapture();
    cam.setResolution(320, 240);
    //was 320 240
    

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
     hoodSolenoid.toggle();
     hoodDown=true;
     if(AutoSelected=="Auto1"){
      AutonomousChosen=1;
      SmartDashboard.putNumber("auto", 1);
    }else if(AutoSelected=="Auto2"){
      AutonomousChosen=2;
      SmartDashboard.putNumber("auto", 2);
    }else if(AutoSelected=="Auto3"){
      AutonomousChosen=3;
      SmartDashboard.putNumber("auto", 3);
    }

     match.start();
     
     gyroStartAngle=navx.getAngle();
     zeroYaw = navx.getAngle() % 360;
     if(zeroYaw < 0){
       zeroYaw += 360;

      autoFrontLeft=frontLeft.getSelectedSensorPosition();
      autoFrontRight=frontRight.getSelectedSensorPosition();
      autoBackLeft=rearLeft.getSelectedSensorPosition();
      autoBackRight=rearRight.getSelectedSensorPosition();
      
      
     }

     gyroStartAngle=navx.getAngle();

    }
    
   

   @Override
   public void autonomousPeriodic() {
    SmartDashboard.putNumber("gyro angle", (navx.getAngle()));
    SmartDashboard.putNumber("gyro start angle", gyroStartAngle);
    SmartDashboard.putNumber("gyro target Angle", gyroTargetAngle);
    SmartDashboard.putNumber("time", tim.get());
     
    gyroTargetAngle=gyroTargetAngle%180;    
    gyroRange=10;
    gyroAngle=navx.getAngle();
    
    //index code
  //   indexSensorValue1 = indexSensor1.get();
  //   sensorProximity = colorSensor.getIR();
    

  //   if(indexSensorValue2==false){
  //     if(indexSensorValue1==false){
  //       if(indexTargetSwitch==false){
  //         indexTargetSwitch=true;
  //         indexTargetCounts=indexMotor.getSelectedSensorPosition()-200000;
          

  //     }
  //   }
  // } 
    
  // if(indexSensorValue2==true){
  //   if(indexSensor2Switch==true){
  //     indexAllow=false;
  //     indexSensor2Switch=false;
  //   }
  // }else{
  //   indexSensor2Switch=true;
  //   indexAllow=true;
  // }

    //shooter code
    if(shooterSpeed == 0.0){
      shooterMotor.set(ControlMode.PercentOutput, 0.0);
    }else{
      shooterMotor.set(ControlMode.Velocity, shooterSpeed);
    }

    if(backspinSpeed==0){
      backspinMotor.set(ControlMode.PercentOutput,0.0);
    }else{
      backspinMotor.set(ControlMode.Velocity, backspinSpeed);
    }
    
    

    if(timStart == false) {
      tim.start();
      timStart = true;
}


    autoAverage=((frontLeft.getSelectedSensorPosition()+frontRight.getSelectedSensorPosition()+rearLeft.getSelectedSensorPosition()+rearRight.getSelectedSensorPosition()-autoFrontLeft-autoFrontRight-autoBackLeft-autoBackRight)/4);


    if(AutoSelected=="Auto1"){
      SmartDashboard.putNumber("auto", 1);
    }else if(AutoSelected=="Auto2"){
      SmartDashboard.putNumber("auto", 2);
    }else if(AutoSelected=="Auto3"){
      SmartDashboard.putNumber("auto", 3);
    }

    if(frontLeftSwitch==false){
      frontLeftSwitch=true;
      autoFrontLeft=frontLeft.getSelectedSensorPosition();
    }
    
    //FIXME AUTO2
    //top auto, start on wall
    if(AutonomousChosen==2){
      
      
      if(tim.get()<1){
        
        shooterSpeed=-14500;
        backspinSpeed=15000;
      }
      if(tim.get()>1&&tim.get()<2){
        if(hoodDown==true){
          hoodSolenoid.toggle();
          hoodDown=false;
        }
        indexMotor.set(ControlMode.PercentOutput, -0.275);
        shooterSpeed=-14500;
        backspinSpeed=15000;
      }
      if(tim.get()>2&&tim.get()<7){
        if(Math.abs(frontLeft.getSelectedSensorPosition())-Math.abs(autoFrontLeft)<45000){
          driveBase.set(-0.1, 0.10, 0);
        }else{
          driveBase.set(0, 0, 0);
        }
        indexMotor.set(ControlMode.PercentOutput, 0);
        shooterSpeed=0.0;
        backspinSpeed=0.0;
        if(pickupDown==false){
          pickupSolenoid.toggle();
          pickupDown=true;
        }
        intakeMotor.set(ControlMode.PercentOutput, 1);
      }
      
      if(tim.get()>7&&tim.get()<9){
        if(Math.abs(frontLeft.getSelectedSensorPosition())-Math.abs(autoFrontLeft)<90000){
          driveBase.set(0.1, -0.1, 0);
        }else{
          driveBase.set(0, 0, 0);
        }
        intakeMotor.set(ControlMode.PercentOutput, 0);
        
      }
      
      
      if(tim.get()>9&&tim.get()<10){
        driveBase.set(0, 0, 0);
        shooterSpeed=-14500;
        backspinSpeed=15000; 
      }
      if(tim.get()>10&&tim.get()<12){
        driveBase.set(0, 0, 0);
        shooterSpeed=-14500;
        backspinSpeed=15000;
        indexMotor.set(ControlMode.PercentOutput, -0.275);
      }
      if(tim.get()>12&&tim.get()<13){
        driveBase.set(0, 0, 0);
        shooterSpeed=-14500;
        backspinSpeed=15000;
        // if(pickupDown==true){
        //   pickupSolenoid.toggle();
        //   pickupDown=false;
        // }
        // if(hoodDown==true){
        //   hoodSolenoid.toggle();
        //   hoodDown=false;
        // }
        indexMotor.set(ControlMode.PercentOutput, -0.275);
        intakeMotor.set(ControlMode.PercentOutput, 0);
      }
    }


    if(AutonomousChosen==3){
        if(timStart == false) {
          tim.start();
          timStart = true;
        }

        // if(tim.get()<1){
        //   pickupSolenoid.toggle();
        //   shooterSpeed=-3000;
        //   backspinSpeed=3000;
        // }
        if(tim.get()>1&&tim.get()<2){
          indexMotor.set(ControlMode.PercentOutput, 0.275);
          
        }

        if(tim.get()>2&&tim.get()<3){
          if(autoAverage>-8181.1){
            driveBase.set(0,-0.10,0);
          }else{
            driveBase.set(0, 0, 0);
          }
        }
        if(tim.get()>3&&tim.get()<4){
          gyroTargetAngle=gyroStartAngle+20;
          if(gyroAngle>(gyroTargetAngle-gyroRange)&gyroAngle<(gyroTargetAngle+gyroRange)){
            driveBase.set(0, 0, 0);
           }else{
            driveBase.set(0,0,0.15);
          }
          
        }
        if(tim.get()>4&&tim.get()<6){
          // if(pickupDown==false){
          //   pickupSolenoid.toggle();
          //   pickupDown=true;
          // }
          if(autoAverage>-77412){
          driveBase.set(0, -0.25, 0);
          }
        }  
        
        if(tim.get()>6&&tim.get()<8){
          // if(pickupDown==true){
          //   pickupSolenoid.toggle();
          //   pickupDown=false;
          // }
          gyroTargetAngle=gyroStartAngle+122;
          if(gyroAngle>(gyroTargetAngle-gyroRange)&gyroAngle<(gyroTargetAngle+gyroRange)){
            driveBase.set(0, 0, 0);
          }else{
            driveBase.set(0,0,0.15);
          }
        }
        if(tim.get()>8&&tim.get()<12){
          // if(pickupDown==false){
          //   pickupSolenoid.toggle();
          //   pickupDown=true;
          // }
          if(autoAverage>-188322.6){
          driveBase.set(0, -0.25, 0);
          }
        }
        if(tim.get()>11&&tim.get()<12){
          // if(pickupDown==true){
          //   pickupSolenoid.toggle();
          //   pickupDown=false;
          // }
          // if(hoodDown==false){
          //   hoodSolenoid.toggle();
          //   hoodDown=true;
          // }
          gyroTargetAngle=gyroStartAngle+76.305;
          if(gyroAngle>(gyroTargetAngle-gyroRange)&gyroAngle<(gyroTargetAngle+gyroRange)){
            driveBase.set(0, 0, 0);
          }else{
            driveBase.set(0,0,0.15);
          }
          shooterSpeed=-3000;
          backspinSpeed=3000;
        }
        if(tim.get()>11&&tim.get()<13){
          shooterSpeed=-3000;
          backspinSpeed=3000;
          indexMotor.set(ControlMode.PercentOutput, 0.275);
        }
        if(tim.get()>13&&tim.get()<15){
          shooterSpeed=0;
          backspinSpeed=0;
          indexMotor.set(ControlMode.PercentOutput, 0);
        }
        // if(hoodDown==true){
        //   hoodSolenoid.toggle();
        //   hoodDown=false;
        // }
        driveBase.set(0, 0, 0);
    }
    //this need indexing to be added
    //remember encoder counts can go down and you have to add them onto the others!!!
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
      //FIXME AUTO1
    if(AutonomousChosen==1){
      
     
    //timer based system
    
      if(tim.get()<1){
        
        shooterSpeed=-14500;
        backspinSpeed=15000;
      }
      if(tim.get()>1&&tim.get()<2){
        if(hoodDown==true){
          hoodSolenoid.toggle();
          hoodDown=false;
        }
        indexMotor.set(ControlMode.PercentOutput, -0.275);
        shooterSpeed=-14500;
        backspinSpeed=15000;
      }
      if(tim.get()>2&&tim.get()<5){
        driveBase.set(0, 0.1, 0);
        shooterSpeed=0.0;
        backspinSpeed=0.0;
        
      }
      if(tim.get()>5&&tim.get()<5.6){
        driveBase.set(0, 0, 0);
      }
  }
  }

  @Override
  public void teleopInit() {
    shooterSpeed=0.0;
    backspinSpeed=0.0;
    timStart = false;
    shootertargetVelocity_UnitsPer100ms=0.0;
    backSpintargetVelocity_UnitsPer100ms=0.0;
    pickupCounter=100;
    indexTargetCounts=indexMotor.getSelectedSensorPosition();
    
    shooterSpeedTemp=15000;

    indexShootToggle=false;
    
    teamSelected = teamChooser.getSelected();
    if(teamSelected=="Blue"){
      teamColor=1;
    }else if(teamSelected=="Red"){
      teamColor=2;
    }else{
      teamColor=3;
    }
  }
    

  @Override
  public void teleopPeriodic() {
  
    teamSelected = teamChooser.getSelected();
    indexSensorValue1 = indexSensor1.get();
    sensorProximity = colorSensor.getIR();
    
    //SmartDashboard.putNumber("Gyro", navx.getAngle());
    // SmartDashboard.putNumber("frontleftrot", frontLeftRot.getSelectedSensorPosition());
    // SmartDashboard.putNumber("FRtrot",  frontRightRot.getSelectedSensorPosition());
    // SmartDashboard.putNumber("BLrot", rearLeftRot.getSelectedSensorPosition());
    // SmartDashboard.putNumber("BR", rearRightRot.getSelectedSensorPosition()%26214);
    SmartDashboard.putNumber("IndexMotorEncoder", indexMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("IndexTargetCounts", indexTargetCounts);
    //SmartDashboard.putBoolean("indexTargetSwitch", indexTargetSwitch);
    //SmartDashboard.putBoolean("indexSensorValue2", indexSensorValue2);
      //SmartDashboard.putBoolean("indexSensorValue1", indexSensorValue1);
      //SmartDashboard.putBoolean("IndexTargetSwitch", indexTargetSwitch);
    

    x = controller.getRawAxis(0);
    y = controller.getRawAxis(1);
    w = controller.getRawAxis(4);
    gyroAngle = navx.getAngle() - zeroYaw;
    gyroAngle %= 360;

    if(gyroAngle < 0) 
      gyroAngle += 360;
    //Sets a deadband of .03 and adjusts the controller values so they don't jump from 0 to .3 and
    //instead are out of the remaining .97 range
    // if(Math.abs(x) < .03)
    //   x = 0.0;
    // else if(x > 0)
    //   x = (x - .05)/.97;
    // else
    //   x = (x + .05)/.97;

    // if(Math.abs(y) < .03)
    //   y = 0.0;
    // else if(y > 0)
    //   y = (y - .05)/.97;
    // else
    //   y = (y + .05)/.97;
    
    // if(Math.abs(w) < 0.01)
    //   w = 0.0;
    // else if(w > 0)
    //   w = (w - .05)/.99;
    // else
    //   w = (w + .05)/.99;
    
    //Squares the values to slow down the speeds closer to the center
    
    

    x *= Math.abs(x)*Math.abs(x)*Math.abs(x);
    y *= Math.abs(y)*Math.abs(y)*Math.abs(y);
    w *= Math.abs(w)*Math.abs(w)*Math.abs(w);
    
    x=x*1;
    y=y*1;
    
    
    if(slowMode) {
      x *= .25;
      y *= .25;
    }

    //w * 0.7 limits rotational speed
    if(fieldOriented)
      driveBase.set(x, y*-1, w*.5, gyroAngle);
    else
      driveBase.set(x, y*-1, w*.5);

    //this gives the triggers boolean outputs, its at 0.05 so it removes noise
    // xbox controller code
    if((Math.abs(controller.getRawAxis(2))>0.05)){
      leftTrigger=true;
    }else{
      leftTrigger=false;
    }
    
    //xbox controller code
    if((Math.abs(controller.getRawAxis(3))>0.05)){
      rightTrigger=true;
    }else{
      rightTrigger=false;
    }
    
    

    //color sensor and prox code
    Color detectedColor = colorSensor.getColor();
    double IR = colorSensor.getIR();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putBoolean("slowmode", slowMode);
    
    //this is just switching the proximity value into a boolean for easy use
    if(sensorProximity>ProximitySwitch){
      indexSensorValue2=true;
    }else{
      indexSensorValue2=false;
    }
    
    //SmartDashboard.putNumber("indexvelocity", indexMotor.getSelectedSensorVelocity());
    //1 is blue, 2 is red
    if(indexSensorValue2==true){
      if(teamColor==1){
        if(detectedColor.red>0.32){
          shooterSpeed=-5000;
          backspinSpeed=5000;
          indexAllow=true;
          if(indexShootToggle==false){
            indexShootToggle=true;
            indexTargetCounts-=20000;
          }else{
            indexShootToggle=false;
          }
          //activate motor at low velocity also do the index
        }else{
          if(rightTrigger==false){
            shooterSpeed=0;
            backspinSpeed=0;
          }
        }
      }else if(teamColor==2){
        if(detectedColor.blue>0.28){
          shooterSpeed=-5000;
          backspinSpeed=5000;
          indexAllow=true;
          if(indexShootToggle==false){
            indexShootToggle=true;
            indexTargetCounts-=20000;
          }else{
            indexShootToggle=false;
          }
        }else{
          if(rightTrigger==false){
            shooterSpeed=0;
            backspinSpeed=0;
          }
        }
      }
    }else if(rightTrigger==false){
      shooterSpeed=0;
      backspinSpeed=0;
    }

    if(teamSelected=="Blue"){
      teamColor=1;
    }else if(teamSelected=="Red"){
      teamColor=2;
    }else{
      teamColor=3;
    }

    //index sensors
      //switch indexsensorvalue2 to true once you have sensor
      if(indexSensorValue2==false){
        if(indexSensorValue1==false){
          if(indexTargetSwitch==false){
            indexTargetSwitch=true;
            indexTargetCounts=indexMotor.getSelectedSensorPosition()-200000;
            

        }
      }
    } 
      
    SmartDashboard.putBoolean("indexSensorValue2", indexSensorValue2);
    SmartDashboard.putBoolean("indexSensorValue1", indexSensorValue1);
    SmartDashboard.putBoolean("indexTargetSwitch", indexTargetSwitch);
    

    if(indexSensorValue2==true){
      if(indexSensor2Switch==true){
        indexAllow=false;
        indexSensor2Switch=false;
      }
    }else{
      indexSensor2Switch=true;
      indexAllow=true;
    }

    SmartDashboard.putBoolean("indexAllow", indexAllow);
    SmartDashboard.putBoolean("indexSensor2Switch", indexSensor2Switch);


    //FIXME index
    if(indexAllow==true){    
      if(Math.abs(Math.abs(indexTargetCounts)-Math.abs(indexMotor.getSelectedSensorPosition()))>300){
        indexMotor.set(ControlMode.Position, indexTargetCounts);
        indexCountsDifference=Math.abs(indexMotor.getSelectedSensorPosition())-Math.abs(indexTargetCounts);
        SmartDashboard.putNumber("IndexSETTER", 1);
      }else{
        indexTargetSwitch=false; 
        indexMotor.set(ControlMode.Position, indexMotor.getSelectedSensorPosition());
        indexTargetCounts=indexMotor.getSelectedSensorPosition();
        SmartDashboard.putNumber("IndexSETTER", 2);
      }
    }else{
      indexMotor.set(ControlMode.Position, indexMotor.getSelectedSensorPosition());
      indexTargetCounts=indexMotor.getSelectedSensorPosition();
      SmartDashboard.putNumber("IndexSETTER", 3);
    }
    

    if(intakeMotor.getSupplyCurrent()>7||agitatorMotor.getSupplyCurrent()>7){
      if(agitatorCurrentSwitch==false){
        agitatorCurrentSwitch=true;
      }
      else{
        agitatorCurrentSwitch=false;
      }
    }

    if(agitatorCurrentSwitch){
      if(agitatorTimerSwitch==false){
        AgitatorTimer.start();
      }     
      
    }else{
      AgitatorTimer.stop();
      AgitatorTimer.reset();
    }

    

    if(AgitatorTimer.get()>0.01&&AgitatorTimer.get()<3){
      agitatorMotor.set(ControlMode.PercentOutput, 0.3);
    }else{
      agitatorMotor.set(ControlMode.PercentOutput, 0.1);
    }


    SmartDashboard.putNumber("indexCurrent", intakeMotor.getSupplyCurrent());
    SmartDashboard.putNumber("AgitatorCurrent", agitatorMotor.getSupplyCurrent());
    //SmartDashboard.putNumber("index velocity", indexMotor.getSelectedSensorVelocity());



    //controller code starts here


    //pickup motor code
    if(leftTrigger==true && letUpLeftTrigger) {
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
        agitatorMotor.set(ControlMode.PercentOutput, -1);
      }else{
        intakeMotor.set(ControlMode.PercentOutput, 0);
      }
      if(leftTrigger==false &&pickupCounter<100){
        pickupCounter++;
      }
      if(pickupCounter<25){
        intakeMotor.set(ControlMode.PercentOutput, 1);
      }
      //SmartDashboard.putNumber("pickupcounter", pickupCounter);
      
      

      //resetting field orient
      if(controller.getRawButton(rightOptionButton) && letUpRightOptions) {
          zeroYaw = navx.getAngle() %360;
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
          pickupSolenoid.toggle();
          pickupDown = true;
        } else {
          pickupSolenoid.toggle();
          pickupDown = false;
        }
        letUpLBump = false;
      } else if(!controller.getRawButton(leftBumper)) {
        letUpLBump = true;
      }  
  
      //Slow Mode
      if(controller.getRawButton(rightJoystickButton)&&letUpRightJoystickButton){
        slowMode = !slowMode;
        letUpRightJoystickButton=false;
      }else if(!controller.getRawButton(rightJoystickButton)){
        letUpRightJoystickButton=true;
      }
  
      if(controller.getRawButton(X)&&letUpX){
        if(climberPinsout==false){
            slowMode=true;
        }
        
        letUpX=false;
      }else if(!controller.getRawButton(X)){
        if(climberPinsout==false){
          slowMode=false;
        }
        letUpX=true;
      }
    
    //Flap
    if(controller.getRawButtonPressed(Y) && letUpY)
    {
      
      if(flapDown == false) {
        flapSolenoid.toggle();
        flapDown = true;
      } else {
        flapSolenoid.toggle();
        flapDown = false;
      }
      letUpY = false;
    } else if(!controller.getRawButton(Y)) {
      letUpY = true;
    }

    //field orient code
    if(controller.getRawButton(leftOptionButton) && letUpLeftOptions) {
      fieldOriented = !fieldOriented;
      letUpLeftOptions = false;
    } else if(!controller.getRawButton(leftOptionButton) &&!letUpLeftOptions) {
      letUpLeftOptions = true;
    }
  
    

    if(controller.getRawButtonPressed(A) && letUpA)
    {
      
      if(hoodDown == false) {
        hoodSolenoid.toggle();
        hoodDown = true;
      } else {
        hoodSolenoid.toggle();
        hoodDown = false;
      }
      letUpA = false;
    } else if(!controller.getRawButton(Y)) {
      letUpA = true;
    }
    
    //brings up flap if above 25% speed
    // if((Math.abs(y)>0.40||Math.abs(x)>0.40)&&pickupDown==true){
    //   pickupSolenoid.toggle();
    //   pickupDown=false;
    // }
    SmartDashboard.putBoolean("hooddown", hoodDown);
    SmartDashboard.putBoolean("fieldOreint", fieldOriented);
    
    //Shooter function
    if(hoodDown==false){
      if(controller.getRawButton(rightBumper) && Math.abs(shooterMotor.getSelectedSensorVelocity()) > 8300&&Math.abs(backspinMotor.getSelectedSensorVelocity())>300){
        indexAllow=true;
        //the if statment makes sure it only does this once until the next ball shoots
        if(indexShootToggle==false){
          indexShootToggle=true;
          indexTargetCounts-=20000;
        }else{
          indexShootToggle=false;
        }
        //1024 is the amount of counts it needs to mode, not tuned
      } 
    }
    if(hoodDown==true){
      if(controller.getRawButton(rightBumper) && Math.abs(shooterMotor.getSelectedSensorVelocity()) > 8600&&Math.abs(backspinMotor.getSelectedSensorVelocity())>300){
      indexAllow=true;
      //the if statment makes sure it only does this once until the next ball shoots
      if(indexShootToggle==false){
        indexShootToggle=true;
        indexTargetCounts-=20000;
      }else{
        indexShootToggle=false;
      }
      //1024 is the amount of counts it needs to mode, not tuned
    }  
    }
     
    

    //shooter warmup
    if(rightTrigger&&letUpRightTrigger){
      if(hoodDown==false){
        shooterSpeed=-14000;
        backspinSpeed=15000;
      }else{
        shooterSpeed=-14500;
        backspinSpeed=15000;
      }
      
      letUpRightTrigger=false;
    }else if((!rightTrigger)&&!letUpRightTrigger){
      letUpRightTrigger=true;
      shooterSpeed=0.0;
      backspinSpeed=0.0;
    }

    


    
    //FIXME timer for climber
    // if(match.get() >= 105){
      
      //climber up
      if(controller.getPOV()==0&&letUpPOV0) {
        slowMode=true;
        letUpPOV0 = false;
            //FIXME CLIMBER CODE
            if(climberPinsout==false){
              climberPinsSolenoid.toggle();
              climberPinsout=true;
              
            }else if(bigclimberdown==true){
              bigClimberSolenoid.toggle();
              bigclimberdown=false;
              
            }

          } else if(controller.getPOV()!=0 && !letUpPOV0) {
            letUpPOV0 = true;
          }
      //b
      //climberDown
      if(controller.getPOV()==180&&letUpPOV180) {
        letUpPOV180 = false;

        if(bigclimberdown==false&&climberPinsout==true){
          bigClimberSolenoid.toggle();
          bigclimberdown=true;
            climberPinsSolenoid.toggle();
        }
      } else if(controller.getPOV()!=180&& !letUpPOV180) {
        letUpPOV180 = true;
      }
  //}
  
  if(Controller2.getPOV()==180&&letUpPOV180C2) {
    letUpPOV180C2 = false;
    climberPinsSolenoid.toggle();
  } else if(Controller2.getPOV()!=180&& !letUpPOV180C2) {
    letUpPOV180C2 = true;
  }

  if(Controller2.getPOV()==0&&letUpPOV0C2) {
    letUpPOV0C2 = false;
    shooterSpeedTemp+=100;
  } else if(Controller2.getPOV()!=180&& !letUpPOV0C2) {
    letUpPOV0C2 = true;
  }
  

    //example
    // if(controller.getRawButton(leftBumper) && letUpBack) {
    //   falcon.set(ControlMode.PercentOutput, 0.1);
    //     letUpBack = false;
    //   } else if(!controller.getRawButton(leftBumper) && !letUpBack) {
    //     letUpBack = true;
    //     falcon.set(ControlMode.PercentOutput, 0.0);
    //   }

    
    

    
    
    
    //TIMER CODE
    

    if(shooterSpeed == 0.0){
      shooterMotor.set(ControlMode.PercentOutput, 0.0);
    }else{
      shooterMotor.set(ControlMode.Velocity, shooterSpeed);
    }
    SmartDashboard.putNumber("velocity", shooterMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("backspinVelocity", backspinMotor.getSelectedSensorVelocity());

    if(backspinSpeed==0){
      backspinMotor.set(ControlMode.PercentOutput,0.0);
    }else{
      backspinMotor.set(ControlMode.Velocity, backspinSpeed);
    }
    

    if(match.get() > 105) {
      //arnold.disable();
    }
    
    
  
    //SmartDashboard commandsf
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