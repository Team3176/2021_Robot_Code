// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Flywheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrenchPass extends InstantCommand {
  Hood mAngledShooter = Hood.getInstance();
  Drum mDrum = Drum.getInstance();
  Transfer mTransfer = Transfer.getInstance();
  Flywheel mFlywheel = Flywheel.getInstance();
  
  public TrenchPass() {
    addRequirements(mAngledShooter);
    addRequirements(mDrum);
    addRequirements(mTransfer);
    addRequirements(mFlywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mAngledShooter.moveTop();
    mDrum.pctCtrl_set(.3);
    mFlywheel.spinVelocityOutputPercent(1);

    Timer.delay(1);

    mTransfer.setPercentControl(0);

    Timer.delay(1);

    mAngledShooter.moveBottom();
    mFlywheel.spinVelocityOutputPercent(0);
    mDrum.pctCtrl_set(0);
  }
}
