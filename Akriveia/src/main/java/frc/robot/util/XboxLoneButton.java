package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxLoneButton extends Trigger {
  private final XboxController m_XboxController;
  private final int m_Include;
  private final int m_Exclude;

  /**
   * Class to create Double button trigger.
   * @param xboxController The joystick
   * @param includeButton Button to be the "lone" button
   * @param excludeButton1 The first button in the pair to exclude
   */ 
  public XboxLoneButton( XboxController xboxController, int includeButton, int excludeButton) {
    this.m_XboxController = xboxController;
    this.m_Include = includeButton;
    this.m_Exclude = excludeButton;
  } 

  @Override
  public boolean get() {
    if (m_XboxController.getRawButton(m_Include) && !m_XboxController.getRawButton(m_Exclude)) {
      return true;
    }
    return false;
  }
}
