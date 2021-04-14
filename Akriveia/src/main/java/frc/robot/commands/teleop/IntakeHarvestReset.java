package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Intake;

/**
 * Shakes the Drum briefly and stops the Intake.
 */
public class IntakeHarvestReset extends InstantCommand {
  Drum mDrum = Drum.getInstance();
  Intake mIntake = Intake.getInstance();

  public IntakeHarvestReset() {
    addRequirements(mDrum);
    addRequirements(mIntake);
  }

  @Override
  public void initialize() {
    // System.out.println("IntakeHarvestReset.initialize executed. ############################################################");
    mDrum.shakeDrum();
    mIntake.setPercentControl(0);
    // System.out.println("SPRING");
  }
}
