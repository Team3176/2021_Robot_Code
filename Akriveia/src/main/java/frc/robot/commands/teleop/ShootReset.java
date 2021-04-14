package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.BallTransfer;

public class ShootReset extends InstantCommand {
  Drum mDrum = Drum.getInstance();
  // Flywheel mFlywheel = Flywheel.getInstance();
  BallTransfer mTransfer = BallTransfer.getInstance();
  
  public ShootReset() {
    addRequirements(mDrum);
    // addRequirements(mFlywheel);
    addRequirements(mTransfer);
  }

  @Override
  public void initialize() {
    // System.out.println("ShootReset.initialize executed. ############################################################");
    mDrum.pidVelCtrl_set(0);
    // mFlywheel.spinVelocityOutputPercent(0);
    mTransfer.setPercentControl(0);
    mTransfer.Retract();
  }
}