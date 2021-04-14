package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drum;

public class DrumSpinReverse extends InstantCommand {

  Drum m_Drum = Drum.getInstance();

  public DrumSpinReverse() {
    addRequirements(m_Drum);
  }

  @Override
  public void initialize() {
    // System.out.println("DrumSpinReverse.initialize executed. ############################################################");
    m_Drum.ShortSpinInReverse();
  }
}
