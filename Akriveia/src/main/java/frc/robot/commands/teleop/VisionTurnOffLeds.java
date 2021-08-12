// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Vision;
import frc.robot.constants.VisionConstants;

// we are in the eighth hole from the front on the LL base plate

public class VisionTurnOffLeds extends InstantCommand{
  
  private Vision m_Vision = Vision.getInstance();

  public VisionTurnOffLeds(){}

  @Override
  public void initialize(){
    //m_Vision.setLedMode(VisionConstants.VISION_LED_OFF);  
    m_Vision.setPipeline(VisionConstants.PIPELINE_FOR_DRIVER_CAM);
  }
}
