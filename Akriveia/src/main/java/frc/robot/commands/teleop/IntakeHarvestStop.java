package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drum;


/**
 * Shakes the Drum briefly and stops the Intake.
 */
public class IntakeHarvestStop extends InstantCommand {
  Drum m_Drum = Drum.getInstance();
 

  public IntakeHarvestStop() {
    addRequirements(m_Drum);

  }

  @Override
  public void initialize() {
    // System.out.println("IntakeHarvestReset.initialize executed. ############################################################");
    m_Drum.stopMotors();
    // mIntake.retractIntake();
    // System.out.println("SPRING");
  }
}
