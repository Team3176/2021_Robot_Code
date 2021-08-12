// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Hood;

public class HoodPosDown extends CommandBase {

  private Hood m_AngledShooter = Hood.getInstance();

  public HoodPosDown() {
    addRequirements(m_AngledShooter);
  }

  @Override
  public void initialize(){
    //System.out.println("----AngledShooter__Down----");
    
  }

  @Override
  public void execute(){
    m_AngledShooter.moveBottom();
    //System.out.println("DOWN__EXE");
  }

  @Override
  public void end(boolean interrupted) {
    //System.out.println("DOWN__END");
    m_AngledShooter.pctCtrl_set(0);
  }

  @Override
  public boolean isFinished() {
    //System.out.println("DOWN__IS");
    if(m_AngledShooter.getBottomSwitch()) return true;
    return false;
  }
}
