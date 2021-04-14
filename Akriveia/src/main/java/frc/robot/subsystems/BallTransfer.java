package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.BallTransferConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Responsible for transferring the Power Cells from the Drum to the AngledShooter. Uses a piston and motor to do so.
 */
public class BallTransfer extends SubsystemBase {
  private static BallTransfer instance = new BallTransfer();
  private DoubleSolenoid transferPiston = new DoubleSolenoid(BallTransferConstants.DS_OPEN_ID, BallTransferConstants.DS_CLOSE_ID);
  private CANSparkMax transferMotor = new CANSparkMax(BallTransferConstants.MOTOR_CAN_ID, MotorType.kBrushless);
  private boolean pistonSetting = false;
  private double levelSetting = 0;

  public BallTransfer() {}

  public static BallTransfer getInstance() {
    return instance;
  }

  /**
   * @param percent - the percent -1 to 1 inclusive of the transferMotor
   */
  public void setPercentControl(double percent) {
    levelSetting = percent;
    transferMotor.set(percent);
  }

  /**
   * Extends the transferPiston
   */
  public void Extend() {
    pistonSetting = true;
    transferPiston.set(Value.kForward);
  }

  /**
   * Retracts the transferPiston
   */
  public void Retract() {
    pistonSetting = false;
    transferPiston.set(Value.kReverse);
  }

  /**
   * @return Gets the piston setting incase its extended or retracted
   */
  public boolean getPistonSetting() {
    return pistonSetting;
  }

  /**
   * @return Gets the percentage of the motor from the last time it was set
   */
  public double getLevelSetting() {
    return levelSetting;
  }
}