// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drum;
import frc.robot.commands.teleop.*;
import frc.robot.commands.auton.*;

/**
 * Shakes the Drum rapidly to dislodge stuck Power Cells.
 * @author Jared Brown
 */
public class DrumAgitate extends SequentialCommandGroup {
  /** Creates a new DrumAgitate. */
  
  public DrumAgitate() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new DrumAgitateStart(),
      new DrumAgitateStop()
    );
  }

}
