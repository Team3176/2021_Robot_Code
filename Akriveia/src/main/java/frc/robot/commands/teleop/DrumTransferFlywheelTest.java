package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Flywheel;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrumTransferFlywheelTest extends CommandBase {
  Drum m_Drum = Drum.getInstance();
  // BallTransfer m_BallTransfer = BallTransfer.getInstance();
  Flywheel m_Flywheel = Flywheel.getInstance();
  
  public DrumTransferFlywheelTest() {
    addRequirements(m_Drum);
    // addRequirements(m_BallTransfer);
    addRequirements(m_Flywheel);
  }

  @Override
  public void initialize() {
    // System.out.println("DrumTransferFlywheelTest.java initialize");
    // double drumOutputPercent = SmartDashboard.getNumber("DrumOutputPercent", 0);
    // double transferOutputPercent = SmartDashboard.getNumber("BallTransferOutputPercent", 0);
    // double flywheelOutputPercent = SmartDashboard.getNumber("FlywheelOutputPercent", 0);
    // m_Drum.pctCtrl_set(drumOutputPercent);
    // m_BallTransfer.setPercentControl(transferOutputPercent);
    // m_Flywheel.spinVelocityOutputPercent(flywheelOutputPercent);
  }

  @Override
  public void execute() {
    // double drumOutputPercent = SmartDashboard.getNumber("DrumOutputPercent", 0);
    // double transferOutputPercent = SmartDashboard.getNumber("BallTransferOutputPercent", 0);
    // double flywheelOutputPercent = SmartDashboard.getNumber("FlywheelOutputPercent", 0);
    // m_Drum.pctCtrl_set(drumOutputPercent);
    // m_BallTransfer.setPercentControl(transferOutputPercent);
    // m_Flywheel.spinVelocityOutputPercent(flywheelOutputPercent);
  }

  @Override
  public boolean isFinished() { 
    // if (SmartDashboard.getBoolean("StopTest06", true)) {
      // return true;
    // } else {
      return false;
    // }
  }

  @Override
  public void end(boolean interrupted) { 
    m_Drum.pctCtrl_set(0.0);
    // m_BallTransfer.setPercentControl(0.0);
    m_Flywheel.spinVelocityOutputPercent(0.0);

  }

}