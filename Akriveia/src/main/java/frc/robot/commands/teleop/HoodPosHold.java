// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Hood;

public class HoodPosHold extends CommandBase {

  private Hood m_AngledShooter = Hood.getInstance();

  public HoodPosHold() {
    addRequirements(m_AngledShooter);
  }

  @Override
  public void initialize(){}

  @Override
  public void execute(){
    //m_AngledShooter.pctCtrl_holdHoodPosition();
    // m_AngledShooter.pctCtrl_set(0.1);
    m_AngledShooter.pctCtrl_set(0);
    m_AngledShooter.setPosToHold();
    m_AngledShooter.posCtrl();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
