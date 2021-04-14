package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AngledShooterConstants;
import frc.robot.subsystems.PowerManagement;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import java.util.ArrayList;

public class AngledShooter extends SubsystemBase {
  
  private TalonSRX hoodController = new WPI_TalonSRX(AngledShooterConstants.MOTOR_CAN_ID);
  private static AngledShooter instance = new AngledShooter();
  private static PowerManagement m_PowerManagement;

  private double hoodDirection;
  private double ampSpikeCount;
  private double startPosTic; 
  private double minPosTic, maxPosTic;
  private int numPositions, hoodPositions_persistingIndex;
  private ArrayList<Double> hoodPositions_Tics; 

  public AngledShooter() {
    boolean pidPosCtrlActive = false;  //set True if doing PID Position Ctrl.  Set False if doing OutputPercent Ctrl.
    boolean pctCtrlActive = ! pidPosCtrlActive;

    m_PowerManagement = PowerManagement.getInstance();
    

	  /* Setting up the Motor */
	  
    hoodController.configFactoryDefault();
    hoodController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, AngledShooterConstants.PIDLoopIdx, AngledShooterConstants.TimeoutMs);
    hoodController.setSensorPhase(AngledShooterConstants.SENSOR_PHASE);
    hoodController.setInverted(AngledShooterConstants.MOTOR_INVERT);
    hoodController.configNominalOutputForward(0, AngledShooterConstants.TimeoutMs);
    hoodController.configNominalOutputReverse(0, AngledShooterConstants.TimeoutMs);
    hoodController.configPeakOutputForward(/*1*/0.2, AngledShooterConstants.TimeoutMs);
    hoodController.configPeakOutputReverse(/*-1*/-0.2, AngledShooterConstants.TimeoutMs);
    hoodController.configAllowableClosedloopError(0, AngledShooterConstants.PIDLoopIdx, AngledShooterConstants.TimeoutMs);
    hoodController.config_kF(AngledShooterConstants.PIDLoopIdx, AngledShooterConstants.PIDF[3], AngledShooterConstants.TimeoutMs);
		hoodController.config_kP(AngledShooterConstants.PIDLoopIdx, AngledShooterConstants.PIDF[0], AngledShooterConstants.TimeoutMs);
		hoodController.config_kI(AngledShooterConstants.PIDLoopIdx, AngledShooterConstants.PIDF[1], AngledShooterConstants.TimeoutMs);
    hoodController.config_kD(AngledShooterConstants.PIDLoopIdx, AngledShooterConstants.PIDF[2], AngledShooterConstants.TimeoutMs);

    ampSpikeCount = 0;
    

    if (pidPosCtrlActive) {
      pidPosCtrl_createPositionsAndGoToInitialPos(); 
    } 
      

    //hoodController.configContinuousCurrentLimit(20);   //using 20amps b/c that's what was shown in DS logs
    //hoodController.configPeakCurrentDuration(1000);   //1000ms .... is totally a guess.
  }

  public void pidPosCtrl_createPositionsAndGoToInitialPos() {
    startPosTic = hoodController.getSelectedSensorPosition();
    minPosTic = startPosTic;
    maxPosTic = startPosTic + 1500;
    numPositions = 0;
    SmartDashboard.putNumber("Hood startPosTic", startPosTic);
    SmartDashboard.putNumber("Hood minPosTic", minPosTic);
    SmartDashboard.putNumber("Hood maxPosTic", maxPosTic);
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
    SmartDashboard.putNumber("maxPosTic_Calculated", maxPosTicCalc);
    SmartDashboard.putNumber("maxPosTic_Sensored", maxPosTic);
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
    SmartDashboard.putNumber("minPosTic_Calculated", minPosTicCalc);
    SmartDashboard.putNumber("minPosTic_Sensored", minPosTic);
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

    SmartDashboard.putNumber("Hood_NumOfPositions", hoodPositions_Tics.size());
    SmartDashboard.putNumberArray("Hood_Positions_in_Tics", tempCandidateHoodPosition);
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
  public static AngledShooter getInstance() {return instance;}

  /**
   * Sets the positon of the Angled Shooter Hood
   * @param targetPosition - Gets the target position that it should go if its inside of the MIN and MAX constants
   * @see commands.teleop.AngledShooterUp
   * @see commands.teleop.AngledShooterDown
   */
  public void pidPosCtrl_setPosition(double targetPosition) {
    if (targetPosition >= AngledShooterConstants.MIN_TICS && targetPosition <= AngledShooterConstants.MAX_TICS){
      hoodController.set(ControlMode.Position, targetPosition);
    }
  }

  public void pctCtrl_set(double pct) {
    hoodController.set(ControlMode.PercentOutput, pct);
  }

  public void pctCtrl_raiseHoodPosition() {
    pctCtrl_set(AngledShooterConstants.MOVE_PCT);
  }

  public void pctCtrl_holdHoodPosition() {
    pctCtrl_set(AngledShooterConstants.HOLD_PCT);
  }
  
  public void pctCtrl_lowerHoodPosition() {
    pctCtrl_set(-AngledShooterConstants.MOVE_PCT);
  }

  public void pctCtrl_stopHoodMotor() {
    pctCtrl_set(0);
  }

	/**
	 * @return the encoder position by using .getSelectedSensorPosition()
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


  @Override
  public void periodic() {
    checkForCurrentSpike();
    SmartDashboard.putNumber("Hood Amps", m_PowerManagement.getAngledShooterAvgAmp());
    SmartDashboard.putNumber("HoodPos_inTics", hoodController.getSelectedSensorPosition());
  }


  //double feedforward = 0.07   // <- determine from Tuner, % output to offset gravity
  //hoodController.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, feedforward);
}