package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.PowerManagement;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import java.util.ArrayList;

public class Hood extends SubsystemBase {
  
  private TalonSRX hoodController = new WPI_TalonSRX(HoodConstants.MOTOR_CAN_ID);
  private static Hood instance = new Hood();
  private static PowerManagement m_PowerManagement;

  private double hoodDirection;
  private double ampSpikeCount;
  private double startPosTic; 
  private double minPosTic = -210;
  private double maxPosTic = -1420;
  private int numPositions, hoodPositions_persistingIndex;
  private ArrayList<Double> hoodPositions_Tics;
  private double[] pos5 = new double[5];
  private int setting;
  private double posToHold;
  private DigitalInput topSwitch;
  private DigitalInput bottomSwitch;

  public Hood() {
    boolean pidPosCtrlActive = false;  //set True if doing PID Position Ctrl.  Set False if doing OutputPercent Ctrl.
    boolean pctCtrlActive = ! pidPosCtrlActive;

    m_PowerManagement = PowerManagement.getInstance();
    
    topSwitch = new DigitalInput(HoodConstants.TOP_LIMIT_SWITCH_DIO_ID);
    bottomSwitch = new DigitalInput(HoodConstants.BOTTOM_LIMIT_SWITCH_DIO_ID);
	  /* Setting up the Motor */
    minPosTic = 4078;
    maxPosTic = 2487;
    
    hoodController.configFactoryDefault();
    hoodController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, HoodConstants.PIDLoopIdx, HoodConstants.TimeoutMs);
    hoodController.setSensorPhase(HoodConstants.SENSOR_PHASE);
    hoodController.setInverted(HoodConstants.MOTOR_INVERT);
    hoodController.configNominalOutputForward(0, HoodConstants.TimeoutMs);
    hoodController.configNominalOutputReverse(0, HoodConstants.TimeoutMs);
    hoodController.configPeakOutputForward(0.25, HoodConstants.TimeoutMs);
    hoodController.configPeakOutputReverse(-0.25, HoodConstants.TimeoutMs);
    hoodController.configAllowableClosedloopError(0, HoodConstants.PIDLoopIdx, HoodConstants.TimeoutMs);
    hoodController.config_kF(HoodConstants.PIDLoopIdx, HoodConstants.PIDF[3], HoodConstants.TimeoutMs);
		hoodController.config_kP(HoodConstants.PIDLoopIdx, HoodConstants.PIDF[0], HoodConstants.TimeoutMs);
		hoodController.config_kI(HoodConstants.PIDLoopIdx, HoodConstants.PIDF[1], HoodConstants.TimeoutMs);
    hoodController.config_kD(HoodConstants.PIDLoopIdx, HoodConstants.PIDF[2], HoodConstants.TimeoutMs);

    // ampSpikeCount = 0;
    // findMinMax();

    // if (pidPosCtrlActive) {
    //   pidPosCtrl_createPositionsAndGoToInitialPos(); 
    // } 
      

    //hoodController.configContinuousCurrentLimit(20);   //using 20amps b/c that's what was shown in DS logs
    //hoodController.configPeakCurrentDuration(1000);   //1000ms .... is totally a guess.
  }

  public void stopMotors() {
    hoodController.set(ControlMode.PercentOutput, 0);
  }

  public void moveTop() {
    // hoodController.set(ControlMode.Position, maxPosTic);
    hoodController.set(ControlMode.PercentOutput, .5);
  }
  
  public void moveBottom() {
    // hoodController.set(ControlMode.Position, minPosTic);
    hoodController.set(ControlMode.PercentOutput, -.5);
  }

  public void findMinMax() { // Finds the min pos and the max pos
    hoodController.set(ControlMode.PercentOutput, 1);
    Timer.delay(1);
    maxPosTic = hoodController.getSelectedSensorPosition();
    System.out.println("------" + minPosTic);
    hoodController.set(ControlMode.PercentOutput, -.4);
    Timer.delay(1);
    hoodController.set(ControlMode.PercentOutput, 0);
    minPosTic = hoodController.getSelectedSensorPosition();
    System.out.println("++++++" + maxPosTic);
    boolean direction = minPosTic - maxPosTic > 0; //TRUE is POS; FALSE is NEG
    double range;
    pos5[0] = minPosTic;
    pos5[4] = maxPosTic;
    setting = 0;

    if(direction) {
      range = maxPosTic - minPosTic;
      pos5[1] = (.25 * range) + minPosTic;
      pos5[2] = (.50 * range) + minPosTic;
      pos5[3] = (.75 * range) + minPosTic;
    } else {
      range = minPosTic - maxPosTic;
      pos5[1] = minPosTic - (range * .25);
      pos5[2] = minPosTic - (range * .50);
      pos5[3] = minPosTic - (range * .75);
    }
    downPos();
  }

  public void downPos() {
    setting = 0;
    hoodController.set(ControlMode.Position, pos5[setting]);
  }

  public void upPos() {
    setting = pos5.length - 1;
    hoodController.set(ControlMode.Position, pos5[setting]);
  }

  public void changePos(int lvl) {
    setting = lvl;
    if(setting >= 0 && setting < pos5.length) {
      hoodController.set(ControlMode.Position, pos5[setting]);
    }
  }

  public int getSetting() {
    return setting;
  }

  public void pidPosCtrl_createPositionsAndGoToInitialPos() {
    startPosTic = hoodController.getSelectedSensorPosition();
    minPosTic = startPosTic;
    maxPosTic = startPosTic + 1500;
    numPositions = 0;
    // SmartDashboard.putNumber("Hood startPosTic", startPosTic);
    // SmartDashboard.putNumber("Hood minPosTic", minPosTic);
    // SmartDashboard.putNumber("Hood maxPosTic", maxPosTic);
    //pidPosCtrl_findMinPosTicMaxPosTic();
    pidPosCtrl_buildHoodPositions(3);
    hoodController.set(ControlMode.Position, hoodPositions_Tics.get(hoodPositions_persistingIndex));
    hoodPositions_Tics = new ArrayList<Double>();
    hoodPositions_persistingIndex = 0;
  }

  public void pidPosCtrl_findMinPosTicMaxPosTic() {
    double targetPos = startPosTic;
   // while (checkForCurrentSpike() == 0) {
      targetPos = targetPos - (350);
      hoodController.set(ControlMode.Position, targetPos);
   // }
    hoodController.set(ControlMode.Position, targetPos - 350);
    double maxPosTicCalc = (targetPos + 350);
    maxPosTic = hoodController.getSelectedSensorPosition();
    // SmartDashboard.putNumber("maxPosTic_Calculated", maxPosTicCalc);
    // SmartDashboard.putNumber("maxPosTic_Sensored", maxPosTic);
    // System.out.println("********** maxPosTic_Calculated (maxPosTicCalc) = " + maxPosTicCalc + "***************************");
    // System.out.println("********** maxPosTic_Sensored (maxPosTic) = " + maxPosTic + "***************************");
    targetPos = maxPosTic;
    //while (checkForCurrentSpike() == 0) {
      targetPos = targetPos + (350);
      hoodController.set(ControlMode.Position, targetPos);
    //}
    hoodController.set(ControlMode.Position, targetPos + 350);
    double minPosTicCalc = (targetPos - 350);
    minPosTic = hoodController.getSelectedSensorPosition();
    // SmartDashboard.putNumber("minPosTic_Calculated", minPosTicCalc);
    // SmartDashboard.putNumber("minPosTic_Sensored", minPosTic);
    // System.out.println("********** minPosTic_Calculated (minPosTicCalc) = " + minPosTicCalc + "***************************");
    // System.out.println("********** minPosTic_Sensored (minPosTic) = " + minPosTic + "***************************");
  }

  public int pidPosCtrl_buildHoodPositions(int numberOfPositionsDesired) {
    numPositions = numberOfPositionsDesired;
    double range = Math.abs(maxPosTic - minPosTic);
    if (range == 0) {
      // System.out.println("*************ERROR: AngledShooter.buildHoodPositions FAILED due to a 0 range between min and max positions of the Hood***************");
      return 0;
    }
    double tempCandidateHoodPosition[] = new double[numberOfPositionsDesired];
    double ticsBetweenHoodPositions = range / (numberOfPositionsDesired - 1);
    int ticsBetweenHoodPositions_truncd = (int)ticsBetweenHoodPositions;
    double candidateHoodPosition = minPosTic;
    hoodPositions_Tics.add(70.0);
    hoodPositions_Tics.add(800.0);
    hoodPositions_Tics.add(1500.0);
    tempCandidateHoodPosition[0]=hoodPositions_Tics.get(0);
    tempCandidateHoodPosition[1]=hoodPositions_Tics.get(1);
    tempCandidateHoodPosition[2]=hoodPositions_Tics.get(2);
    hoodPositions_persistingIndex = 0;

    
    /*
    for(int idx = 0; idx <= (int)numberOfPositionsDesired-1 ; idx++) {
      hoodPositions_Tics.add(candidateHoodPosition);
      candidateHoodPosition += ticsBetweenHoodPositions_truncd;
      tempCandidateHoodPosition[idx] = hoodPositions_Tics.get(idx);
    }
    */

    // SmartDashboard.putNumber("Hood_NumOfPositions", hoodPositions_Tics.size());
    // SmartDashboard.putNumberArray("Hood_Positions_in_Tics", tempCandidateHoodPosition);
    return 1;
  }

  public void pidPosCtrl_goUpToNextHoodPosition_Tic(){
    if (hoodPositions_Tics.get(hoodPositions_persistingIndex+1) < hoodController.getSelectedSensorPosition() ) {
      hoodPositions_persistingIndex += 1;
      hoodController.set(ControlMode.Position, hoodPositions_Tics.get(hoodPositions_persistingIndex));
    } else { 
      // System.out.println("*************ERROR:  AngledShooter.goUpToNextHoodPosition :: Already at top of possible Hood positions *****************************");
    }
  }

  public void pidPosCtrl_goUpToNextHoodPosition_Tic_version2(){
    if (hoodPositions_persistingIndex + 1 < numPositions ) {
      hoodPositions_persistingIndex += 1;
      hoodController.set(ControlMode.Position, hoodPositions_Tics.get(hoodPositions_persistingIndex));
    } else { 
      // System.out.println("*************ERROR:  AngledShooter.goUpToNextHoodPosition :: Already at top of possible Hood positions *****************************");
    }
  }
  
  public void goUpToNextHoodPosition(double pct) {
    hoodController.set(ControlMode.PercentOutput, pct);
  }
  
  public void pidPosCtrl_goDownToNextHoodPosition_Tic(){
    if (hoodPositions_persistingIndex - 1 >= 0) {
      hoodPositions_persistingIndex -= 1;
      hoodController.set(ControlMode.Position, hoodPositions_Tics.get(hoodPositions_persistingIndex));
    } else { 
      // System.out.println("*************ERROR:  AngledShooter.goDownToNextHoodPosition :: Already at bottom of possible Hood positions *****************************");
    }
  }


  /**
   * @return A single, universal Intake instance to be used anywhere else in the code */
  public static Hood getInstance() {return instance;}

  /**
   * Sets the positon of the Angled Shooter Hood
   * @param targetPosition - Gets the target position that it should go if its inside of the MIN and MAX constants
   * @see HoodPosUp.teleop.AngledShooterUp
   * @see HoodPosDown.teleop.AngledShooterDown
   */
  public void pidPosCtrl_setPosition(double targetPosition) {
    if (targetPosition >= HoodConstants.MIN_TICS && targetPosition <= HoodConstants.MAX_TICS){
      hoodController.set(ControlMode.Position, targetPosition);
    }
  }

  public void pctCtrl_set(double pct) {
    hoodController.set(ControlMode.PercentOutput, pct);
  }

  public void pctCtrl_raiseHoodPosition() {
    pctCtrl_set(HoodConstants.MOVE_PCT);
  }

  public void pctCtrl_holdHoodPosition() {
    pctCtrl_set(HoodConstants.HOLD_PCT);
  }
  
  public void pctCtrl_lowerHoodPosition() {
    pctCtrl_set(-HoodConstants.MOVE_PCT);
  }

  public void pctCtrl_stopHoodMotor() {
    pctCtrl_set(0);
    //pidCtrl_holdPosition();
  }

  public int pidCtrl_holdPosition() {
    // double pos2Hold = SmartDashboard.getNumber("Hood Position (Tics)", 0);
    double pos2Hold = 0;
    if(pos2Hold == 0) {return 1;}
    hoodController.set(ControlMode.Position, pos2Hold);
    return 0;
  }

  public void setPosToHold() {
    posToHold = hoodController.getSelectedSensorPosition();
  }

  public void posCtrl() {
    // double currPos = SmartDashboard.getNumber("Hood Position (Tics)", hoodController.getSelectedSensorPosition());
    hoodController.set(ControlMode.Position, /*currPos*/posToHold);
  }

	/**
	 * @return the hood motor's encoder position by using .getSelectedSensorPosition()
	 */
  public double getEncoderPosition() {
    return hoodController.getSelectedSensorPosition();
  }

  public int checkForCurrentSpike() {
    int returnFlag = 0;
    double amps = m_PowerManagement.getAngledShooterAvgAmp();
    if (amps >= 20) {
      ampSpikeCount++;
      hoodDirection = hoodDirection * -1;
      returnFlag = 1;
    }
    if (ampSpikeCount > 5) {
      hoodController.set(ControlMode.PercentOutput, 0);
      ampSpikeCount = 0;
      returnFlag = 2;
    }
    return returnFlag;
  }

  public void setPID(double kP, double kI, double kD) {
		hoodController.config_kP(HoodConstants.PIDLoopIdx, kP, HoodConstants.TimeoutMs);
		hoodController.config_kI(HoodConstants.PIDLoopIdx, kI, HoodConstants.TimeoutMs);
    hoodController.config_kD(HoodConstants.PIDLoopIdx, kD, HoodConstants.TimeoutMs);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Hood Amps", m_PowerManagement.getAngledShooterAvgAmp());
    // SmartDashboard.putNumber("Hood Position (Tics)", hoodController.getSelectedSensorPosition());
    // if(!topSwitch.get()) System.out.println("Top: " + topSwitch.get());
    // if(!bottomSwitch.get()) System.out.println("Bottom: " + bottomSwitch.get());
  }

  public boolean getBottomSwitch() {
    //System.out.println("Bottom Switch: " + bottomSwitch.get());
    return !bottomSwitch.get();
  }

  public boolean getTopSwitch() {
    //System.out.println("Top Switch: " + topSwitch.get());
    return !topSwitch.get();
  }

  //double feedforward = 0.07   // <- determine from Tuner, % output to offset gravity
  //hoodController.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, feedforward);
}