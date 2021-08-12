package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Flywheel;

public class FlywheelVelocityDown extends InstantCommand {

  Flywheel m_Flywheel = Flywheel.getInstance();

  public FlywheelVelocityDown() {
    addRequirements(m_Flywheel);
  }

  @Override
  public void initialize() {
    // System.out.println("FlywheelSlow.initialize executed. ############################################################");
    /* Finds whatever setting the intake pistons are and does the opposite. */

    int tempSetting = m_Flywheel.getLastSetting();
    if (tempSetting - 1 >= 0) {
      m_Flywheel.spinVelocityPIDF(tempSetting - 1);
    }
  }
}