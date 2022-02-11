package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.*;

//import com.revrobotics.CANPIDController;


// import com.revrobotics.CANPIDController;

// import com.revrobotics.CANSparkMax;
//import com.revrobotics.ControlType;
// import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRF_Swerve_Module {    
    TalonFX rotationMotor;
    TalonFX speedMotor;
    AnalogInput encoder;
    
    //CANPIDController speedPID;
    
    double speedP = 5e-5, speedI = 1e-6, speedD = 0;
    double zeroOffset;
    final int countsPerRev = 2048;

    private double PIDTarget;

    public SRF_Swerve_Module(int encoderID,int rotID, int driveID, double P, double I, double D, double offset) {
        encoder = new AnalogInput(encoderID);
        
        rotationMotor = new TalonFX(rotID);
        
        rotationMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0, 0);
        rotationMotor.config_kP(0, P);
        rotationMotor.config_kI(0, I);
        rotationMotor.config_kD(0, D);
        
        speedMotor = new TalonFX(driveID);
        
        speedMotor.setNeutralMode(NeutralMode.Brake);
        speedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        speedMotor.config_kP(0,speedP);
        speedMotor.config_kI(0,speedI);
        speedMotor.config_kD(0,speedD);
        speedMotor.setSensorPhase(true);
        speedMotor.overrideLimitSwitchesEnable(false);
        speedMotor.overrideSoftLimitsEnable(false);
        speedMotor.enableVoltageCompensation(true);
        speedMotor.configOpenloopRamp(0.08);
        speedMotor.configVoltageCompSaturation(12);
        speedMotor.enableVoltageCompensation(true);
        speedMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 0.5),0);
        
        
        
        
        // speedMotor = new CANSparkMax(sparkID, MotorType.kBrushless);
        // speedMotor.setIdleMode(IdleMode.kBrake);
        // speedMotor.setSmartCurrentLimit(50);
        // speedMotor.setOpenLoopRampRate(.35);
        // speedPID = new CANPIDController(speedMotor);
        // speedPID.setP(speedP);
        // speedPID.setI(speedI);
        // speedPID.setD(speedD);
        //the 12.8 is the gear ratio modify if changes
        //this value exists cause the if statment would not take math.abs
        double offsetDifference;
        
        offsetDifference=offset-encoder.getVoltage();
        if(Math.abs(offsetDifference)>2.5){
            if(offsetDifference>0){
            offsetDifference=-5+offsetDifference;
            }else if(offsetDifference<0){
                offsetDifference=5-Math.abs(offsetDifference);
            }
        }
        SmartDashboard.putNumber("offset"+encoderID, offset);
        SmartDashboard.putNumber("encoderCount"+encoderID, encoder.getVoltage());
        SmartDashboard.putNumber("offsetDifference"+encoderID, offsetDifference);
        zeroOffset=offsetDifference;
        //zeroOffset=Math.abs(offset-encoder.getVoltage());
        //rotationMotor.set(ControlMode.Position, rotationMotor.getSelectedSensorPosition() + Math.abs((encoder.getVoltage()-zeroOffset)*26214.4));
        



    }

    public void set(double angle, double speed) {
        SmartDashboard.putNumber(("encoder"+encoder.getChannel()), encoder.getVoltage());
        //SmartDashboard.updateValues();


        //SmartDashboard.putNumber("angle", angle);
        //SmartDashboard.putNumber(("encoder"+encoder),rotationMotor.getSelectedSensorPosition());
        int currentAngle = (rotationMotor.getSelectedSensorPosition());
        double distanceBetween;
        int sign = 1;
        double gearratio=12.8;
        
        //SmartDashboard.putNumber("currentangle1", currentAngle%26214);
        currentAngle %= (countsPerRev*gearratio);
        
        if(angle < 0){
            currentAngle += (countsPerRev*gearratio);
        }    
        //SmartDashboard.putNumber("% counts/Rev", currentAngle);

        angle = (angle/360*-1)*(2048*gearratio);
        SmartDashboard.putNumber("differnecebetween"+encoder.getChannel(), (rotationMotor.getSelectedSensorPosition()%26214.4)-(rotationMotor.getSelectedSensorPosition()-angle));
        angle += (zeroOffset*(5242.88));
        //multipied by 5400
        //SmartDashboard.putNumber("angle before adding",angle);
        
        if(angle < 0){
            angle += (countsPerRev*gearratio);
        }
        //SmartDashboard.putNumber("Angle in Rev", angle);
        
        distanceBetween = angle - currentAngle;
        //SmartDashboard.putNumber("FirstDistBetween", distanceBetween);
        if(distanceBetween < 0)
            distanceBetween += (countsPerRev*gearratio);
        //SmartDashboard.putNumber("Init DistBetween", distanceBetween);
        
        if(distanceBetween > ((countsPerRev*gearratio) - distanceBetween)) {
            distanceBetween = (countsPerRev*gearratio) - distanceBetween;
            sign *= -1;
        }


        if(distanceBetween > (512*gearratio)) {
            distanceBetween = (1024*gearratio) - distanceBetween;
            sign *= -1;
            speed *= -1;
            //SmartDashboard.putNumber("distBetween Changed", distanceBetween);
        }

        
        //reduces deadzones a little
        if(speed==0){
            
        }else if(speed>0){
            speed+=0.056;
        }else if(speed<0){
            speed-=0.056;
        }
        
        //SmartDashboard.putNumber("speed", speed);
        SmartDashboard.putNumber("zerooffset"+encoder.getChannel(), zeroOffset*5400);
        if(Math.abs(distanceBetween) > (10*gearratio)){    
            rotationMotor.set(ControlMode.Position, (rotationMotor.getSelectedSensorPosition() + distanceBetween* sign ));
        }
        
        //speedPID.setReference(speed, ControlType.kDutyCycle);
        //SmartDashboard.putNumber("sign", sign);
        //SmartDashboard.putNumber("Value", rotationMotor.getSelectedSensorPosition() + distanceBetween * sign);
        speedMotor.set(ControlMode.PercentOutput, speed*0.5);
        //SmartDashboard.putNumber("Distance Between", distanceBetween);
        //SmartDashboard.putNumber("MotorPosition",rotationMotor.getSelectedSensorPosition());
        PIDTarget = rotationMotor.getSelectedSensorPosition() - distanceBetween;
        
    }

    public double getPIDTarget() {
        return PIDTarget;
    }
    
    public int getSensorValue() {
        return rotationMotor.getSelectedSensorPosition();
    }

    public double getMotorPosition(){
        return speedMotor.getSelectedSensorPosition();

    }
}