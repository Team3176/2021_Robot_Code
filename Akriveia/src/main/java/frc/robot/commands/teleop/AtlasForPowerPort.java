// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Flywheel;
import frc.robot.constants.VisionConstants;

// we are in the eighth hole from the front on the LL base plate

public class AtlasForPowerPort extends CommandBase {
  
  private Vision m_Vision = Vision.getInstance();
  private Flywheel m_Flywheel = Flywheel.getInstance();
  private Hood m_AngledShooter = Hood.getInstance();
  private String zone;
  private boolean isHoodUp, onBlueYellowLine;

  public AtlasForPowerPort(){}

  @Override
  public void initialize(){
    addRequirements(m_Flywheel);
    addRequirements(m_AngledShooter);

    isHoodUp = true;
    //m_AngledShooter.pctCtrl_raiseHoodPosition();
    m_AngledShooter.pctCtrl_holdHoodPosition();
    m_Vision.setAtlasOn(true);  
  }

  @Override
  public void execute(){
    if ((m_Vision.getDeltaX() > 3.8) && (m_Vision.getDeltaX() <= 4.4)) {    // <- Not sure about the 10.5 & 14.5.  Using these as initial guess based on 12.5 (where Blue & Yellow meet) with +/- 2.0
      // SmartDashboard.putBoolean("ATLAS PowerPort Fire Signal", true);
      onBlueYellowLine = true;
    } else {
      // SmartDashboard.putBoolean("ATLAS PowerPort Fire Signal", false);
      onBlueYellowLine = false;
    }
    zone = m_Vision.findShootingZone();
    if(zone.equals("GREEN")){
      m_Flywheel.setRpm(3000);
      /*if(isHoodUp){
        m_AngledShooter.pctCtrl_lowerHoodPosition();
        isHoodUp = false;
      }*/
    } else if((zone.equals("YELLOW")) && (!onBlueYellowLine)){
      m_Flywheel.setRpm(3850);
      if(!isHoodUp){
        m_AngledShooter.pctCtrl_raiseHoodPosition();
        m_AngledShooter.pctCtrl_holdHoodPosition();
        isHoodUp = true;
      }
    } else if( (zone.equals("BLUE")) && (!onBlueYellowLine)){
      m_Flywheel.setRpm(4000);
      if(!isHoodUp){
        m_AngledShooter.pctCtrl_raiseHoodPosition();
        m_AngledShooter.pctCtrl_holdHoodPosition();
        isHoodUp = true;
      }
    } else if(zone.equals("RED")){
      m_Flywheel.setRpm(4300);
      if(!isHoodUp){
        m_AngledShooter.pctCtrl_raiseHoodPosition();
        m_AngledShooter.pctCtrl_holdHoodPosition();
        isHoodUp = true;
      }
    } else if (onBlueYellowLine) {
      m_Flywheel.setRpm(3800);
      if(!isHoodUp){
        m_AngledShooter.pctCtrl_raiseHoodPosition();
        m_AngledShooter.pctCtrl_holdHoodPosition();
        isHoodUp = true;
      }
    } else{
      m_Flywheel.setRpm(0);
      /*if(isHoodUp){
        m_AngledShooter.pctCtrl_lowerHoodPosition();
        isHoodUp = false;
      }*/
    }
  }

  @Override
  public void end(boolean interrupted){
    if(isHoodUp){
      m_AngledShooter.pctCtrl_lowerHoodPosition();

    }
    m_Flywheel.setRpm(0);
  }

  @Override
  public boolean isFinished(){
    return !m_Vision.isAtlasOn();

  }
}
