package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LoneButton extends Trigger {
  private final Joystick m_joystick;
  private final int m_include;
  private final int m_exclude;

  /**
   * Class to create Double button trigger.
   * @param joystick The joystick
   * @param includeButton Button to be the "lone" button
   * @param excludeButton1 The first button in the pair to exclude
   * @param excludeButton2 The second button in the pair to exclude
   */ 
  public LoneButton(Joystick joystick, int includeButton, int excludeButton) {
    this.m_joystick = joystick;
    this.m_include = includeButton;
    this.m_exclude = excludeButton;
  } 

  @Override
  public boolean get() {
    if (m_joystick.getRawButton(m_include) && !m_joystick.getRawButton(m_exclude)) {
      return true;
    }
    return false;
  }
}
