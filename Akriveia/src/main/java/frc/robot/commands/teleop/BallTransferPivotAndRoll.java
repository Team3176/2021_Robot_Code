package frc.robot.commands.teleop;

import frc.robot.subsystems.BallTransfer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.BallTransferConstants;

public class BallTransferPivotAndRoll extends InstantCommand {
  
  private BallTransfer m_BallTransfer = BallTransfer.getInstance();
  
  public BallTransferPivotAndRoll() {
    addRequirements(m_BallTransfer);
  }

  @Override
  public void initialize() {
    // System.out.println("BallTransferPivotAndRoll.initialize executed.##########################");
    m_BallTransfer.Retract();
    m_BallTransfer.setPercentControl(0);
  }
}