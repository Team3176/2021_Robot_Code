// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drum;
import frc.robot.constants.DrumConstants;

import java.util.UUID;
public class DrumVelocitySpeed extends CommandBase {
  /** Creates a new DrumVelocitySpeed2. */
  Drum m_Drum = Drum.getInstance();
  int tempSetting;
  String procTag;

  public DrumVelocitySpeed() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drum);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("DrumVelocitySpeed.initialized executed. ########################################################");
    tempSetting = m_Drum.getLastSetting();
    procTag = UUID.randomUUID().toString();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (tempSetting + 1 < DrumConstants.SPEEDS.length) {
      m_Drum.pidVelCtrl_setRpmLevel(tempSetting + 1, procTag);
      //m_Drum.pidVelCtrl_step4LevelsToDesiredSpeed(tempSetting + 1, 1, procTag);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {    
    return (procTag == m_Drum.getProcTag()) ? true : false;
  }
}
