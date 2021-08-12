// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drum;

/**
 * Shakes the Drum rapidly to dislodge stuck Power Cells.
 * @author Jared Brown
 */
public class DrumAgitateStart extends CommandBase {
  /** Creates a new DrumAgitate. */
  Drum m_Drum = Drum.getInstance();
  private boolean ranThroughSequence;
  
  public DrumAgitateStart() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drum);
  }

  @Override
  public void initialize() {
    // System.out.println("DrumAgitate.initialize executed. ############################################################");
  }

  @Override
  public void execute() {
    ranThroughSequence = m_Drum.shakeDrum();
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
