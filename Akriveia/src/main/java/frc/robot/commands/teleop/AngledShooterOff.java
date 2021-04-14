package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AngledShooter;

public class AngledShooterOff extends InstantCommand {
  AngledShooter mAngledShooter = AngledShooter.getInstance();

  public AngledShooterOff() {
    addRequirements(mAngledShooter);
  }

  @Override
  public void initialize() {
    // System.out.println("&&&&&&&&&&&&&&&&&&_ANGLED_SHOOTER_0FF_&&&&&&&&&&&&&&&&&&");
    mAngledShooter.pctCtrl_set(0.0);
  }
}
