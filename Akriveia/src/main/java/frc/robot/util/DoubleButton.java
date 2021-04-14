package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DoubleButton extends Trigger {
  private final Joystick m_joystick;
  private final int m_button1;
  private final int m_button2;

  /**
   * Class to create Double button trigger.
   * @param joystick The joystick
   * @param button1 The first button in the pair
   * @param button2 The second button in the pair
   */ 
  public DoubleButton(Joystick joystick, int button1, int button2) {
    this.m_joystick = joystick;
    this.m_button1 = button1;
    this.m_button2 = button2;
  } 

  @Override
  public boolean get() {
    return m_joystick.getRawButton(m_button1) && m_joystick.getRawButton(m_button2);
  } 
}

/* Use like below:
 *
 *  private Joystick joystick = new Joystick(ControllerConstants.JoystickID); 
 *  private Button button = new DualButton(joystick, 1, 2);
 *
 *
 *   button.whenPressed(new MyCommand());
 */

