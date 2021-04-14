package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxAxisAsButton extends Trigger {
  private final XboxController m_XboxController;
  private final int m_Axis;
  private final double m_Threshold;

  /**
   * Class to use one of the Xbox "Axis" inputs as a trigger/button.
   * Axis inputs on XboxController include: 
   *     1.) Left & Right triggers on back of controller have 1 axis each, ranging from 0 to 1. Default value = 0
   *     2.) Left & Right joysticks on controller have 2 axises each: 
   *          i. Y axis (ie LYAxis & RYAxis) ranging from -1 to 1.  Default value = 0
   *          ii. X axis (ie LXAxis & RXAxis) ranging from -1 to 1.  Default value = 0
   * 
   * @param xboxController The joystick
   * @param axis Axis input to convert into a button. 
   * @param threshold The axis value above which will return a "button press", and below which will return no "button press"
   */ 
  public XboxAxisAsButton( XboxController xboxController, int axis, double threshold) {
    this.m_XboxController = xboxController;
    this.m_Axis = axis;
    this.m_Threshold = threshold;
  } 

  @Override
  public boolean get() {
    if (m_XboxController.getRawAxis(m_Axis) >= m_Threshold) {
      return true;
    }
    return false;
  }
}
