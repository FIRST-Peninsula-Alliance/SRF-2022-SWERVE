package frc.robot;

//import edu.wpi.first.wpilibj.interfaces.Gyro;

//import javax.lang.model.util.ElementScanner6;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

class SRF_Swerve_Drive {
    
    private double wheelBase, trackWidth, radius;
    //autoDistance in inches, use decimal
    private double AutoAngle, AutoSpeed, AutoDistance;
    private double AutoMotorTemp = 0.0;
    private double AutoMotorRotations,WheelRadius, AutoX,AutoY, AutoSpeedMaxer, RevolutionsperWheelSpin;
    private double AutoMotorRotationsTotal=0;
    private double AutoDriveTempRemovable;
    Double autoDriftX;
    Double autoDriftY;
    Double gyroStartAngle;

    private static boolean AutoDriveCompletion=false;

    //private boolean AutoRemovableCompletion;

    private static boolean AutoDriveCalculationSwitch=false;

    private double[] AutoXList = new double[100];
    private double[] AutoYList = new double[100];
    private double[] AutoMotorRotationsList = new double[100];
    private double[] AutoAngleList = new double[100];
    private double[] AutoSpeedList = new double[100];
    private double[] AutoDistanceList = new double[100];
    
   

    private double[] wheelSpeed = new double[4];
    private double[] wheelAngle = new double[4];

    private SRF_Swerve_Module frontLeftModule;
    private SRF_Swerve_Module frontRightModule;
    private SRF_Swerve_Module rearLeftModule;
    private SRF_Swerve_Module rearRightModule;

    AHRS navx;

    public SRF_Swerve_Drive(SRF_Swerve_Module frontLeft, SRF_Swerve_Module frontRight, SRF_Swerve_Module rearLeft,
                            SRF_Swerve_Module rearRight, double w, double t, double r) {
        frontLeftModule = frontLeft;
        frontRightModule = frontRight;
        rearLeftModule = rearLeft;
        rearRightModule = rearRight;
        wheelBase = w;
        trackWidth = t;
        radius = r;
    }

    /*
    Changes the X,Y, values inputed (Like from a controller joystick) so they are oriented 
    to forward being the opposite end of the field from you as opposed to the front of the robot
    */
	private double[] convertToFieldPosition(double X, double Y, double gyroAngle){
        double newY, newX, temp;
        
        gyroAngle = (gyroAngle/180) * Math.PI;
        //SmartDashboard.putNumber("angle at Convert", gyroAngle);

        temp = Y*Math.cos(gyroAngle) + X*Math.sin(gyroAngle);
        newX = Y*-1*Math.sin(gyroAngle) + X*Math.cos(gyroAngle);
        newY = temp;
        //SmartDashboard.putNumber("newX", newX);
        //SmartDashboard.putNumber("newY", newY);

        return new double[] {newX,newY};
    }

    //Sets and runs the calculate function (see below) with the appropriate X, Y, and Rotation
    public void set(double X, double Y, double W) {
        calculate(X,Y,W);
    }

    //Does the same as above except changing the values to be field oriented
    public void set(double X, double Y, double W, double gyroAngle) {
        double[] newCoordinates = convertToFieldPosition(X,Y,gyroAngle);
        X = newCoordinates[0];
        Y = newCoordinates[1];
        calculate(X,Y,W);
    }
    
    private void calculate(double X, double Y, double W){
        double A, B, C, D;
        double greatestValue = -1;
        
        A = X - W*(wheelBase/radius);  
        B = X + W*(wheelBase/radius);
        C = Y - W*(trackWidth/radius);
        D = Y + W*(trackWidth/radius);

        wheelSpeed[0] = (Math.sqrt(Math.pow(B,2) + Math.pow(C,2)));
        wheelSpeed[1] = Math.sqrt(Math.pow(B,2) + Math.pow(D,2));
        wheelSpeed[2] = Math.sqrt(Math.pow(A,2) + Math.pow(D,2));
        wheelSpeed[3] = Math.sqrt(Math.pow(A,2) + Math.pow(C,2));
            
        //if no controller input keep current angle(Math inside would change) but change speed to zero(Math above)
        if(X != 0 || Y != 0 || W != 0) {
            /*
            Finds the largest wheel speed greater then 1 and converts all speeds so they are proportional
            to the largest being 1. It does this since the range of the motor is [-1,1].
            Note: the values from the equation above for the wheel speed are only positive
            */ 
            for(int wheel = 0; wheel < 4; wheel++) {
                if(wheelSpeed[0] > 1 && wheelSpeed[0] > greatestValue) {
                    greatestValue = wheelSpeed[0];
                }
            }
            if(greatestValue > -1) {
                for(int wheel = 0; wheel < 4; wheel++) {
                    wheelSpeed[0] = wheelSpeed[0]/greatestValue;
                }
            }

            //gives angle of the wheels in degrees
            wheelAngle[0] = Math.atan2(B,C)*(180/Math.PI);
            wheelAngle[1] = Math.atan2(B,D)*(180/Math.PI);
            wheelAngle[2] = Math.atan2(A,D)*(180/Math.PI);
            wheelAngle[3] = Math.atan2(A,C)*(180/Math.PI);
        }
        frontRightModule.set(wheelAngle[0], wheelSpeed[0]);
        frontLeftModule.set(wheelAngle[1], wheelSpeed[1]);
        rearLeftModule.set(wheelAngle[2], wheelSpeed[2]);
        rearRightModule.set(wheelAngle[3], wheelSpeed[3]);
    }

    public void setSpeedZero(){
        wheelSpeed[0] = 0;
        wheelSpeed[1] = 0;
        wheelSpeed[2] = 0;
        wheelSpeed[3] = 0;
    }

    public void displaySmartDashboard(boolean showAngle, boolean showSpeed, boolean showRotEncoder, 
                                      boolean showPIDTarget){
        if(showAngle) {
            //Angle in degrees
            SmartDashboard.putNumber("Front Left Angle", getFrontLeftAngle());
            SmartDashboard.putNumber("Front Right Angle", getFrontRightAngle());
            SmartDashboard.putNumber("Rear Left Angle", getRearLeftAngle());
            SmartDashboard.putNumber("Rear Right Angle",getRearRightAngle());
        }
        if(showSpeed) {
            SmartDashboard.putNumber("Front Left Speed", getFrontLeftSpeed());
            SmartDashboard.putNumber("Front Right Speed", getFrontRightSpeed());
            SmartDashboard.putNumber("Rear Left Speed", getRearLeftSpeed());
            SmartDashboard.putNumber("Rear Right Speed", getRearRightSpeed());
        }
        if(showRotEncoder) {
            SmartDashboard.putNumber("Front Left Sensor", frontLeftModule.getSensorValue());
            SmartDashboard.putNumber("Front Right Sensor", frontRightModule.getSensorValue());
            SmartDashboard.putNumber("Rear Left Sensor", rearLeftModule.getSensorValue());
            SmartDashboard.putNumber("Rear Right Sensor", rearRightModule.getSensorValue());
        }
        if(showPIDTarget) {
            SmartDashboard.putNumber("Front Left PID Target", frontLeftModule.getPIDTarget());
            SmartDashboard.putNumber("Front Right PID Target", frontRightModule.getPIDTarget());
            SmartDashboard.putNumber("Rear Left PID Target", rearLeftModule.getPIDTarget());
            SmartDashboard.putNumber("Rear Right PID Target", rearRightModule.getPIDTarget());
        }
    }

    public double[] getWheelAngles(){
        return new double[] {wheelAngle[0], wheelAngle[1], wheelAngle[2], wheelAngle[3]};
    }

    public double getFrontLeftSpeed(){
        return wheelSpeed[1];
    }

    public double getFrontRightSpeed(){
        return wheelSpeed[0];
    }

    public double getRearLeftSpeed(){
        return wheelSpeed[2];
    }

    public double getRearRightSpeed(){
        return wheelSpeed[3];
    }

    public double getFrontLeftAngle(){
        return wheelAngle[1];
    }

    public double getFrontRightAngle(){
        return wheelAngle[0];
    }

    public double getRearLeftAngle(){
        return wheelAngle[2];
    }

    public double getRearRightAngle(){
        return wheelAngle[3];
    }

    public static boolean getAutoDriveCompletion(){
        return AutoDriveCompletion;

    }

    public static boolean getSwerveDriveCalculation(){
        return AutoDriveCalculationSwitch;

    }

    public static void setSwerveDriveCalculation(boolean cond){
        AutoDriveCalculationSwitch=cond;

    }

    public static void setSwerveDriveCompletion(boolean cond){
        AutoDriveCompletion=cond;
    }

    //AA=AutoAngle,AS=AutoSpeed,AD=AutoDistance;
    public void AutoDriveCalculation(double AA,double AS,double AD,int i){
        
        AutoAngle=AA;
        AutoSpeed=AS;
        AutoDistance=AD;
        WheelRadius=2;
        //change to 1024? 1024 was the original but has been updated to 128 for testing purposes. 
        RevolutionsperWheelSpin=6.00159435;
        
        
        //Changes degrees to radians
        AutoAngle*=(Math.PI/180);
        //This sets the two cordinates to points between -1 and 1
        AutoX=Math.cos(AutoAngle);
        AutoY=Math.sin(AutoAngle);
        //AutoSpeed
        //Normalization of cordinates
        //Takes the two points and multiplies them by t1.5708he speed multipier, so if half speed multiplies by 0.5 and if full multiplies by 1 and does not change
        if(Math.abs(AutoX)>=0.99999){
            //max speed
        }else if(Math.abs(AutoY)>=0.99999){
            //max speeD  
            //these if statments make it so one cordinate always has 1 in it, so it can achive max speed
        }else if(Math.abs(AutoY)>Math.abs(AutoX)){
            AutoSpeedMaxer=1/AutoY;
            AutoSpeedMaxer=Math.abs(AutoSpeedMaxer);
            AutoY=AutoY*AutoSpeedMaxer;
            AutoX=AutoX*AutoSpeedMaxer;
        }else {
            AutoSpeedMaxer=1/AutoX;
            AutoSpeedMaxer=Math.abs(AutoSpeedMaxer);
            AutoY=AutoY*AutoSpeedMaxer;
            AutoX=AutoX*AutoSpeedMaxer;
        }
        //This mulitplies the cordinates by the speed modifier
        AutoX=AutoX*AutoSpeed;
        AutoY=AutoY*AutoSpeed;
        for(int j=0;j<AutoMotorRotationsList.length;j++){
            AutoMotorRotationsTotal=AutoMotorRotationsTotal+AutoMotorRotationsList[j];
        }
        
        //AutoDistance
        //this is calculating the amount of times the motor needs to rotate,
        //it takes the distance and divides it by the wheels circumfrence and then multiplies it by the amount of rpms for one wheel rotation
        double circumference = Math.PI*(2*WheelRadius);
        AutoMotorRotations=(AutoDistance/circumference)*RevolutionsperWheelSpin;
        AutoMotorRotations=AutoMotorRotations+AutoMotorRotationsTotal;
        SmartDashboard.putNumber("AutoXNumber", AutoX);
        SmartDashboard.putNumber("AutoYNumber", AutoY);
        SmartDashboard.putNumber("AutoMotorRotations",AutoMotorRotations);
        AutoAngleList[i]=AutoAngle;
        AutoSpeedList[i]=AutoAngle;
        AutoDistanceList[i]=AutoDistance;
        AutoXList[i]=AutoX;
        AutoYList[i]=AutoY;
        AutoMotorRotationsList[i]=AutoMotorRotations;
        gyroStartAngle=navx.getAngle();
        AutoDriveTempRemovable=frontLeftModule.getMotorPosition();
        
    }

    public void DriftCalculation(double AA,double AS,double AD){
        
        AutoAngle=AA;
        AutoSpeed=AS;
        AutoDistance=AD;
        
        
        
        //Changes degrees to radians
        AutoAngle*=(Math.PI/180);
        //This sets the two cordinates to points between -1 and 1
        AutoX=Math.cos(AutoAngle);
        AutoY=Math.sin(AutoAngle);
        //AutoSpeed
        //Normalization of cordinates
        //Takes the two points and multiplies them by t1.5708he speed multipier, so if half speed multiplies by 0.5 and if full multiplies by 1 and does not change
        if(Math.abs(AutoX)>=0.99999){
            //max speed
        }else if(Math.abs(AutoY)>=0.99999){
            //max speeD  
            //these if statments make it so one cordinate always has 1 in it, so it can achive max speed
        }else if(Math.abs(AutoY)>Math.abs(AutoX)){
            AutoSpeedMaxer=1/AutoY;
            AutoSpeedMaxer=Math.abs(AutoSpeedMaxer);
            AutoY=AutoY*AutoSpeedMaxer;
            AutoX=AutoX*AutoSpeedMaxer;
        }else {
            AutoSpeedMaxer=1/AutoX;
            AutoSpeedMaxer=Math.abs(AutoSpeedMaxer);
            AutoY=AutoY*AutoSpeedMaxer;
            AutoX=AutoX*AutoSpeedMaxer;
        }
        //This mulitplies the cordinates by the speed modifier
        AutoX=AutoX*AutoSpeed;
        AutoY=AutoY*AutoSpeed;
        autoDriftX=AutoX;
        autoDriftY=AutoY;
    }
        
        

       

    public void AutoDrive(int i){
        int holder = i;
        double AutoDriveDriftFixerAngle=navx.getAngle();
        AutoMotorTemp=frontLeftModule.getMotorPosition()-AutoDriveTempRemovable;
        AutoMotorTemp=Math.abs(AutoMotorTemp);
        if(AutoMotorTemp<AutoMotorRotationsList[i]&&AutoMotorTemp>=AutoMotorRotationsList[i-1]){
             if(AutoDriveDriftFixerAngle>gyroStartAngle){
                DriftCalculation(AutoDriveDriftFixerAngle-gyroStartAngle, AutoSpeedList[holder], AutoDistanceList[holder]);
                set(AutoXList[holder]-autoDriftX,AutoYList[holder]-autoDriftY,0);
                //switch the plus or minus if drift is doubled
            }else if(AutoDriveDriftFixerAngle<gyroStartAngle){
                DriftCalculation(AutoDriveDriftFixerAngle+gyroStartAngle, AutoSpeedList[holder], AutoDistanceList[holder]);
                set(AutoXList[holder]+autoDriftX,AutoYList[holder]+autoDriftY,0);
            }
            //set(AutoXList[holder],AutoYList[holder],0);

        }
        SmartDashboard.putNumber("Holder", holder);
        SmartDashboard.putNumber("AutoMotorTemp",AutoMotorTemp);
        
    }

    public void AutoDriveStop(int i){
        int holder=i;
        AutoMotorTemp=frontLeftModule.getMotorPosition()-AutoDriveTempRemovable;
        AutoMotorTemp=Math.abs(AutoMotorTemp);
        if(AutoMotorTemp>=AutoMotorRotationsList[holder-1]-0.5){
            //set(0,0,0);
            setSpeedZero();
        }
    }

    
    
    // public void AutoDrive()
    // {
        
    //     if(AutoMotorTemp<AutoMotorRotations){
    //         SRF_Swerve_Drive.setSwerveDriveCompletion(false);
    //         AutoMotorTemp=frontLeftModule.getMotorPosition()-AutoRemovable;
    //         set(AutoX,AutoY,0);
            
    //     }else{
    //         set(0,0,0);
    //         SRF_Swerve_Drive.setSwerveDriveCompletion(true);
    //     }  
        
    // }
    //FP=first point,SP=second point,TP=Third Point,AS=autospeed
    //public void SwerveAutoSemiCirle(Double FP,SP,Radius,AS){
        //AutoSpeed=AS
        



    //}

    
}