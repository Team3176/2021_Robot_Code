package frc.robot.commands.teleop;

import frc.robot.constants.TransferConstants;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TransferDown extends CommandBase {

  private Transfer m_Transfer = Transfer.getInstance();
  private boolean didTransferExtend; 
  private Vision m_Vision = Vision.getInstance();
  
  public TransferDown() {
    addRequirements(m_Transfer);
  }

  @Override
  public void initialize() {
    // System.out.println("BallTranferStraight.initialized executed. ########################################################");
  }

  @Override
  public void execute(){
    if(m_Vision.getTv()){
    didTransferExtend = m_Transfer.Extend();
    m_Transfer.setPercentControl(TransferConstants.BALL_TRANSFER_PERCENT);
    } else {
      didTransferExtend = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
   
  }

  @Override
  public boolean isFinished() {
    return didTransferExtend;
  }
}

