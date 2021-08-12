package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drum;

/**
 * Shakes the Drum rapidly to dislodge stuck Power Cells.
 * @author Jared Brown
 */
public class DrumAgitateStop extends InstantCommand{
  /** Creates a new DrumAgitate. */
  Drum m_Drum = Drum.getInstance();
  
  public DrumAgitateStop() {
    addRequirements(m_Drum);
  }

  @Override
  public void initialize() {
    // System.out.println("DrumAgitate.initialize executed. ############################################################");
    m_Drum.resetShakeVariables();
  }
}
