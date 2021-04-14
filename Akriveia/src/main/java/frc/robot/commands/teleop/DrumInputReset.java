// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drum;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 * Interrupts the Drum's velocity increment command when changing speeds. (Prevents bugs)
 * @author Jared Brown
 * @see commands.teleop.DrumVelocitySlow
 * @see commands.teleop.DrumVelocitySpeed
 */
public class DrumInputReset extends InstantCommand {
  Drum m_Drum = Drum.getInstance();
  public DrumInputReset() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drum);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("DrumInputReset.initialize executed. ############################################################");
    // Made simply to interrupt the DrumVelocitySlow or DrumVelocitySpeed commands, so that they
    // re-initialize when the same button is pressed multiple times in a row. If they don't
    // re-initialize, the index retrieved from the Drum speeds array will not increase, since
    // the statement getting the previous setting in in the initialize methods so that it doesn't
    // go up on each iteration of those commands.

  }
}
