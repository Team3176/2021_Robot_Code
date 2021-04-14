package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Intake;

public class IntakeHarvest extends InstantCommand {
  Drum mDrum = Drum.getInstance();
  Intake mIntake = Intake.getInstance();

  public IntakeHarvest() {
    addRequirements(mDrum);
    addRequirements(mIntake);
  }

  @Override
  public void initialize() {
    // System.out.println("IntakeHarvest.initialize executed. ############################################################");
    mDrum.CounterClockwise();
    mIntake.setPercentControl(IntakeConstants.INTAKE_PERCENT);
    // System.out.println("HARVEST");
  }
}
