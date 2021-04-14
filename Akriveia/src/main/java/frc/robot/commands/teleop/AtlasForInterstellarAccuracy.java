// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.VisionClient;
import frc.robot.subsystems.AngledShooter;
import frc.robot.subsystems.Flywheel;

// we are in the eighth hole from the front on the LL base plate

public class AtlasForInterstellarAccuracy extends CommandBase {
  
  private VisionClient m_VisionClient = VisionClient.getInstance();
  private Flywheel m_Flywheel = Flywheel.getInstance();
  private AngledShooter m_AngledShooter = AngledShooter.getInstance();
  private String zone;
  private boolean isHoodUp;

  public AtlasForInterstellarAccuracy(){}

  @Override
  public void initialize(){
    addRequirements(m_Flywheel);
    addRequirements(m_AngledShooter);

    m_AngledShooter.pctCtrl_raiseHoodPosition();
    isHoodUp = true;
    m_AngledShooter.pctCtrl_holdHoodPosition();
    m_VisionClient.setAtlasOn(true);  
  }

  @Override
  public void execute(){
    zone = m_VisionClient.findShootingZone();
    if(zone.equals("GREEN")){
      m_Flywheel.setVisionCtrlRPM(3000);
      if(isHoodUp){
        m_AngledShooter.pctCtrl_lowerHoodPosition();
        isHoodUp = false;
      }
    } else if(zone.equals("YELLOW")){
      m_Flywheel.setVisionCtrlRPM(3850);
      if(!isHoodUp){
        m_AngledShooter.pctCtrl_raiseHoodPosition();
        m_AngledShooter.pctCtrl_holdHoodPosition();
        isHoodUp = true;
      }
    } else if(zone.equals("BLUE")){
      m_Flywheel.setVisionCtrlRPM(4000);
      if(!isHoodUp){
        m_AngledShooter.pctCtrl_raiseHoodPosition();
        m_AngledShooter.pctCtrl_holdHoodPosition();
        isHoodUp = true;
      }
    } else if(zone.equals("RED")){
      m_Flywheel.setVisionCtrlRPM(4300);
      if(!isHoodUp){
        m_AngledShooter.pctCtrl_raiseHoodPosition();
        m_AngledShooter.pctCtrl_holdHoodPosition();
        isHoodUp = true;
      }
    } else{
      m_Flywheel.setVisionCtrlRPM(0);
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
    m_Flywheel.setVisionCtrlRPM(0);
  }

  @Override
  public boolean isFinished(){
    return !m_VisionClient.isAtlasOn();

  }
}
