package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.constants.IntakeConstants;

/**
 * <b> The Intake subsystem </b>
 * <p>
 * The mechanism that pulls power cells from the ground and deposits them into the Drum.
 * @see subsystems.Drum
 */
public class Intake extends SubsystemBase {
  private static Intake instance = new Intake();
  private WPI_TalonSRX motor = new WPI_TalonSRX(IntakeConstants.MOTOR_CAN_ID);
  private double intakeMotorSpeed = 0;

  /**
   * Instantiates the Intake object
   */
  public Intake() {}

  /**
   * @return A single, universal Intake instance to be used anywhere else in the code
   */
  public static Intake getInstance() {
    return instance;
  }

  /**
   * Sets the speed of the intake wheels.
   * @param percent between -1 and 1
   */
  public void setPercentControl(double percent) {
    motor.set(ControlMode.PercentOutput, percent);
    intakeMotorSpeed = percent;
    // System.out.println("Intake Percent = " + intakeMotorSpeed + " and " + percent);
  }

  /**
   * @return the intake's motor speed
   */
  public double getIntakeMotorSpeed() {
    // System.out.println("Returning the Intake's Speed");
    return intakeMotorSpeed;
  }
}