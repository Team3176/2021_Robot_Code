// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drum;

/**
 * Shakes the Drum fewer times than DrumAgitate.
 * @see commands.teleop.DrumAgitate
 */
public class DrumAgitatePreShoot extends CommandBase {
  /** Creates a new DrumAgitatePreShoot. */
  Drum m_Drum = Drum.getInstance();
  private boolean ranThroughSequence;

  public DrumAgitatePreShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    // System.out.println("DrumAgitatePreShoot.initialize executed. ############################################################");
  }

  @Override
  public void execute() {
    ranThroughSequence = m_Drum.PreShootSpinAgitate();
  }

  @Override
  public void end(boolean interrupted) {
    m_Drum.resetShakeVariables();
  }

  @Override
  public boolean isFinished() {
    return ranThroughSequence;
  }
}
