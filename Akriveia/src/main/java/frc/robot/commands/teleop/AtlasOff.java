// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Vision;

// we are in the eighth hole from the front on the LL base plate

public class AtlasOff extends InstantCommand{
  
  private Vision m_Vision = Vision.getInstance();

  public AtlasOff(){}

  @Override
  public void initialize(){
    m_Vision.setAtlasOn(false);  
  }
}
