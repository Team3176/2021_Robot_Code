package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Flywheel;
import frc.robot.constants.FlywheelConstants;

public class FlywheelSpeed extends InstantCommand {

  Flywheel m_Flywheel = Flywheel.getInstance();

  public FlywheelSpeed() {
    addRequirements(m_Flywheel);
  }

  @Override
  public void initialize() {

    /* Gets the current speed of the Flywheel and sets it one speed higher */
    // System.out.println("FlywheelSpeed.initialize executed. ##################################################");
    int tempSetting = m_Flywheel.getLastSetting();
    if (tempSetting + 1 < FlywheelConstants.SPEEDS.length) {
      m_Flywheel.spinVelocityPIDF(tempSetting + 1);
    }
  }
}