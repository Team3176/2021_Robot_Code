package frc.robot.commands.teleop;

import frc.robot.constants.BallTransferConstants;
import frc.robot.subsystems.BallTransfer;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class BallTransferStraight extends InstantCommand {

  private BallTransfer m_BallTransfer = BallTransfer.getInstance();
  
  public BallTransferStraight() {
    addRequirements(m_BallTransfer);
  }

  @Override
  public void initialize() {
    // System.out.println("BallTranferStraight.initialized executed. ########################################################");
    m_BallTransfer.Extend();
    m_BallTransfer.setPercentControl(BallTransferConstants.BALL_TRANSFER_PERCENT);
  }
}
