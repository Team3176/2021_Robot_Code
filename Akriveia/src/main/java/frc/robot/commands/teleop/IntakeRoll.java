package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.constants.IntakeConstants;

public class IntakeRoll extends InstantCommand {

  Intake m_Intake = Intake.getInstance();

  public IntakeRoll() {
    addRequirements(m_Intake);
  }

  @Override
  public void initialize() {
    // System.out.println("IntakeRoll.initialize executed. ############################################################");
    /* Gets the current intake speed and sets to the opposite */

    if (m_Intake.getIntakeMotorSpeed() == 0) {
      m_Intake.setPercentControl(IntakeConstants.INTAKE_PERCENT);
      // System.out.println("The intake is set to: " + IntakeConstants.INTAKE_PERCENT);
    } else {
      m_Intake.setPercentControl(0);
      // System.out.println("The intake is 0");
    }
  }
}