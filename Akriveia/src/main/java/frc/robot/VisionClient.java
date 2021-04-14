package frc.robot;

import java.util.Arrays;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.constants.VisionConstants;

/**
 * The VisionClient class is used as a proxy between direct Limelight values and the rest of the Java code.
 * The class also calculates some specific values when used in the context of the 2020/2021 season, such as:
 * -distance from target
 * -horizontal distance from target
 * -vertical distance from target
 * -initial velocity of a ball in order to shoot it into the target
 * -initial angle of the ball in order to shoot it into the target
 */
public class VisionClient{
    // variables to access values given from the Limelight interface
    public NetworkTableInstance tableInstance;
    public NetworkTable limelightTable;
    public NetworkTableEntry tv;
    public NetworkTableEntry tx;
    public NetworkTableEntry ty;
    public NetworkTableEntry tshort;
    public NetworkTableEntry tlong;
    public NetworkTableEntry thor;
    public NetworkTableEntry tvert;
    public NetworkTableEntry tcornxy;
    private NetworkTableEntry tl;
    private NetworkTableEntry pipeline;
    private NetworkTableEntry camMode;
    private NetworkTableEntry ledMode;

    public double hasTarget;
    public double ballAngle;
    
    private double activePipeline = 1;

    // separation of tcornxy into two different arrays
    private double[] tcornx = new double[4];
    private double[] tcorny = new double[4];

    private double deltaXCam;
    private double radius;

    // initializing variables for kinematic calculations
    private final double gravity = -9.81; // m/s^2
    private double deltaX; // m
    private double deltaY; // m
    private double[] initialVelocity = {7089, 6734, 6380, 6025, 5671, 5317, 4962, 4608, 4253, 3899, 3545}; // rpm
    private double initialTheta; // radians
    private double finalTheta; // radians
    private double xVelocity; // m/s
    private double initialYVelocity; // m/s
    private double finalYVelocity; // m/s
    private double time; // seconds

    private int ballLocation = -999; // -999=no ball detected, 0=ball to left, 1=ball exactly 0 degrees forward, 2=ball to right
    private double ballDegrees = -999; // degrees away from Limelight where ball is located. Positive = to left. Negative = to right. Zero = straight ahead.

    private static VisionClient instance = new VisionClient();

    //ADA = Automated Drive Assistant (For used with Drivetrain & Swerve)
    private boolean isAdaOn = false;
    //ATLAS = Advanced Targeting Launch Assistance System (For use with Shooting subsystems, excluding Swerve)
    private boolean isAtlasOn = false;

    /**
     * Creates the default references for VisionClient, specifically for Limelight values
     */
    public VisionClient(){
        tableInstance = NetworkTableInstance.getDefault();
        limelightTable = tableInstance.getTable("limelight");
        updateVisionData();

        pipeline.setNumber(activePipeline);
    }

    public static VisionClient getInstance() {
        return instance;
    }

    /**
     * Can be called to force update of VisionClient data structure
     */
    public void updateVisionData(){
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        tshort = limelightTable.getEntry("tshort");
        tlong = limelightTable.getEntry("tlong");
        thor = limelightTable.getEntry("thor");
        tvert = limelightTable.getEntry("tvert");
        tcornxy = limelightTable.getEntry("tcornxy");
        tl = limelightTable.getEntry("tl");
        pipeline = limelightTable.getEntry("pipeline");
        camMode = limelightTable.getEntry("camMode");
        ledMode = limelightTable.getEntry("ledMode");
        activePipeline = pipeline.getDouble(0);
    }

    public void update(){
        pipeline.setNumber(1);
        targetRecogControlLoop();

        SmartDashboard.putString("Viz_ZONE", findShootingZone());
        SmartDashboard.putNumber("Viz_deltaX (m)", deltaX);
        SmartDashboard.putNumber("Viz_deltaX (ft)", deltaX / VisionConstants.FEET2METER);
        SmartDashboard.putNumber("Viz_initialTheta", initialTheta);
    }
    
    /**
     * All calculations for everything are done in this method.
     * It is essentially the main method in this class.
     * It calculates the following values:
     * -distance of camera from target
     * -horizontal distance of camera from target
     * -vertical distance of camera from target
     * -initial angle of ball to make it into target
     * -initial velocity of ball to make it into target
     * @return if the method has found a valid solution
     */
    public double[] targetRecogControlLoop(){
        // used to calculate latency
        double startTime = Timer.getFPGATimestamp();

        publishPrelimTargetRecogData();

        if(tcornxy.getDoubleArray(new double[1]).length != 8){
            return null;
        }

        separateTcornxyArrayInto2();

        calcTargetRecogDistances();

        publishTargetRecogDistances();

        // get the initial velocity and angle of ball
        double[] resultArray = findInitialAngleAndVelocity(0);
        if (resultArray == null) return null;

        SmartDashboard.putNumber("Latency (ms)", ((Timer.getFPGATimestamp() - startTime) * 1000) + tl.getDouble(0) + 11);
        return resultArray;
    }


    /**
     * Publishes "Has Targets", tshort, tvert, and tcornxy (under "Length" keyname value) to SmartDashboard
     */
    public void publishPrelimTargetRecogData(){
        SmartDashboard.putBoolean("Has Targets", (tv.getDouble(2) == 1) ? true : false);
        SmartDashboard.putNumber("tshort", tshort.getDouble(0));
        SmartDashboard.putNumber("tvert", tvert.getDouble(0));
        SmartDashboard.putNumber("Length", tcornxy.getDoubleArray(new double[1]).length);
    }
    /** 
     * Deconvolutes tcornxy array into tcornx[] and tcorny[] arrays of double datatype.
     */
    public void separateTcornxyArrayInto2(){
        // separate the larger tcornxy arrays into the easier to understand x and y arrays
        int j = 0;
        for(int i = 0; i < 8; i++){
            if(i % 2 == 0){
                tcornx[j] = tcornxy.getDoubleArray(new double[1])[i];
            } else{
                tcorny[j] = tcornxy.getDoubleArray(new double[1])[i];
                j++;
            }
        }
    }

    /**
     * Calculates distances (deltaX and deltaY) to target from Target Recog Data
     */ 
    public void calcTargetRecogDistances(){
        
        // calculate the distance between the furthest two points as the camera sees it
        deltaXCam = calculateDeltaX(tcornx);

        SmartDashboard.putNumber("DeltaXCam", deltaXCam);

        // calculate the various kinds of distances from the camera
        radius = VisionConstants.VISION_CONSTANT / deltaXCam;
        deltaX = radius * Math.cos(ty.getDouble(0) * VisionConstants.DEG2RAD);
        deltaY = radius * Math.sin(ty.getDouble(0) * VisionConstants.DEG2RAD);
    }

    /**
     * Publishes Target Recog Data to SmartDashboard.  Variables published are: radius, deltaX, deltaY.
     */
    public void publishTargetRecogDistances(){
        SmartDashboard.putNumber("Radius", radius);
        SmartDashboard.putNumber("deltaX", deltaX);
        SmartDashboard.putNumber("deltaY", deltaY);
    }
    
    /**
     * Calculates the difference in the two points furthest away from each other
     * 
     * @param array array to find the range of
     * @return double that represents the range of the array
     */
    public double calculateDeltaX(double[] array){
        Arrays.sort(array);
        return array[array.length - 1] - array[0];
    }


    /**
     * Calculates the initial angle and velocity of the ball using kinematic equations
     * 
     * @param speedIdx The index in the possible speed array (above) for the method to start at. It is reccommended to default to 0.
     * @return A double array where the first value is the velocity, and the second value is the angle.
     */
    public double[] findInitialAngleAndVelocity(int speedIdx){
        if(speedIdx >= initialVelocity.length){
            speedIdx = 0;
        }

        findInitialAngle(initialVelocity[speedIdx] * VisionConstants.RPM2MPS);
        publishPrelimTargetRecogData();

        if(!(initialTheta >= 0 && initialTheta <= (29.2 * VisionConstants.DEG2RAD))){
            if(speedIdx < initialVelocity.length - 1){
                return findInitialAngleAndVelocity(speedIdx + 1);
            } else{
                return null;
            }
        }

        solveOtherVariablesFromAngle();

        // figures out if the solution is valid by checking if it would actually go into the target
        if(finalTheta >= (11 * Math.PI)/12 && finalTheta <= (13 * Math.PI)/12){
            double[] arrayToSend = {initialVelocity[speedIdx]  * VisionConstants.RPM2MPS, initialTheta};
            return arrayToSend;
        } else{
            if(speedIdx < initialVelocity.length - 1){
                return findInitialAngleAndVelocity(speedIdx + 1);
            } else {
                return null;
            }
        }
    }


    /**
     * Calculates the initial angle of the ball using kinematic equations
     * 
     * @param speedIdx The index in the possible speed array (above) for the method to start at. It is reccommended to default to 0.
     */
    public void findInitialAngle(double speed){
        // this single line that calculates the initial angle
        initialTheta = Math.atan((-deltaX + Math.sqrt(Math.pow(deltaX, 2) - 4 * ((gravity * deltaX) / (2 * Math.pow(speed, 2))) * (((gravity * deltaX) / (2 * Math.pow(speed, 2))) - deltaY))) / ((gravity * deltaX) / Math.pow(speed, 2)));
    }


    /** 
     * Publishes initialAngle to SmartDashboard under key value "initialTheta".
     */
    public void publishInitialTheta(){
        SmartDashboard.putNumber("initialTheta", initialTheta);
    }


    /**
     * Calculates the other variables needed to verify the solution
     */
    public void solveOtherVariablesFromAngle(){
        // this section calculates the angle that the ball would approach the target to see if it would actually go in
        xVelocity = Math.cos(initialTheta);
        initialYVelocity = Math.sin(initialTheta);

        time = ((-initialYVelocity - Math.sqrt(Math.pow(initialYVelocity, 2) - 4 * (.5 * gravity) * (-1 * deltaY))) / (gravity));

        finalYVelocity = initialYVelocity + (gravity * time);

        finalTheta = Math.atan(finalYVelocity / xVelocity) + Math.PI;
    }


    /**
     * Determines if a ball is detected, and if so how many degrees to left or right of Limelight crosshairs.
     * Negative degrees = Ball to Right of crosshairs.
     * Positive degrees = Ball to Left of crosshairs.
     * Zero degrees = Ball straight ahead forward.
     */
    public void controlLoopBallRecog(){
        pipeline.setNumber(2);

        hasTarget = tv.getDouble(0);
        ballAngle = tx.getDouble(0);

        if(hasTarget == 1){
          SmartDashboard.putBoolean("Ball Recognized", true);
          if(ballAngle <= 1 && ballAngle >= -1){
            ballLocation = 1;
            ballDegrees = 0;
            SmartDashboard.putNumber("Degrees", 0);
          } else if(ballAngle >= 1){
            ballLocation = 2;
            ballDegrees = ballAngle;
              //Ball on the Right side of Limelight crosshairs by absOffset Degrees.
            SmartDashboard.putNumber("Ball Degrees", ballDegrees);
          } else if(ballAngle <= -1){
            ballLocation = 0;
            ballDegrees = ballAngle;
              // Ball on the Left side of Limelight crosshairs by absOffset Degrees.
            SmartDashboard.putNumber("Ball Degrees", ballDegrees);
          } else{
            ballDegrees = -999;
            // System.out.println("Ball Recog FAILED:  see VisionClient.controlLoopBallRecog.");
          }
        } else if(hasTarget == 0){
            ballLocation = -999;
            SmartDashboard.putBoolean("Ball Recognized", false);
        } else{
        //   System.out.println("Ball Recog FAILED:  see VisionClient.controlLoopBallRecog.");
        }
    }

    public String findShootingZone(){
        if(deltaX > 0 && deltaX <= 7.5 * VisionConstants.FEET2METER){
            return "GREEN";
        } else if(deltaX > 7.5 * VisionConstants.FEET2METER && deltaX <= 12.5 * VisionConstants.FEET2METER){
            return "YELLOW";
        } else if(deltaX > 12.5 * VisionConstants.FEET2METER && deltaX <= 17.5 * VisionConstants.FEET2METER){
            return "BLUE";
        } else if(deltaX > 17.5 * VisionConstants.FEET2METER && deltaX <= 22.5 * VisionConstants.FEET2METER){
            return "RED";
        } else if(deltaX > 22.5 * VisionConstants.FEET2METER && deltaX <= 32.5 * VisionConstants.FEET2METER){
            return "RE-INTRODUCTION";
        }
        return "NULL";
    }

    /**
     * Turns on Limelight's LEDs.  Duh.
     */
    public void turnLEDsOn(){
        ledMode.setNumber(3);
    }

    /**
     * Blinks Limelight's LEDs.  Double duh.
     */
    public void blinkLEDs(){
        ledMode.setNumber(2);
    }

    /**
     * Seriously, man? Method's name says it all. It turns off Limelight's LEDs
     */
    public void turnLEDsOff(){
        ledMode.setNumber(1);
    }

    /**
     * Sets camera's mode
     * @param mode determines which mode the method is set to, 0 is vision processing, 1 is Driver Cam, and anything else will print an error message.
     */
    public void setCameraMode(boolean mode){
        if(mode){
            camMode.setNumber(0);
        } else{
            camMode.setNumber(1);
        }
    }

   /**
   * Gets which pipeline the processor will use. Returns double value indicating number of currently active pipeline. 
   */
    public Double getActivePipeline(){
        activePipeline = pipeline.getDouble(0);
        return activePipeline;
    }

   /**
   * Sets which pipeline the processor will use.
   * @param desiredPipelineNum sets the pipeline that will be used. Acceptable values are 0 and 1 at present.
   */
    public void setActivePipeline(double desiredPipelineNum){
        if(desiredPipelineNum == 0 || desiredPipelineNum == 1){
            activePipeline = desiredPipelineNum;
            pipeline.setNumber(activePipeline);
        } 
    }

    public double getBallLocation(){
        return ballLocation;
    }

    public double getBallDegrees(){
        return ballDegrees;
    }

    public double getTargetDistanceX() {
        return deltaX;
    }

    public double getTargetAngle() {
        return initialTheta;
    }

    public boolean getAdaState(){
        return isAdaOn;
    }

    public void setAdaOn(boolean value){
        isAdaOn = value;
    }
    
    public boolean isAdaOn(){
        return isAdaOn;
    }

    public void setAtlasOn(boolean value){
        isAtlasOn = value;
    }
    
    public boolean isAtlasOn(){
        return isAtlasOn;
    }


    public double getDeltaX(){
        return deltaX;
    }
}