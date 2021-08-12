package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.TransferConstants;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Vision;

/**
 * Shoots a power cell, taking control of the Flywheel, Drum, and BallTransfer
 */
public class Shoot extends InstantCommand {
  Drum mDrum = Drum.getInstance();
  Flywheel mFlywheel = Flywheel.getInstance();
  Transfer mTransfer = Transfer.getInstance();
  Vision m_Vision = Vision.getInstance();
  double visionAngle, visionDistanceX;
  
  public Shoot() {
    addRequirements(mDrum);
    addRequirements(mTransfer);
    addRequirements(mFlywheel);
  }

  @Override
  public void initialize() {
    visionAngle = m_Vision.getTargetAngle();
    visionDistanceX = m_Vision.getTargetDistanceX();
    mDrum.pidVelCtrl_setRpmLevel(1);
    mFlywheel.spinVelocityPIDF(8);
    mTransfer.setPercentControl(TransferConstants.BALL_TRANSFER_PERCENT);
  }
}