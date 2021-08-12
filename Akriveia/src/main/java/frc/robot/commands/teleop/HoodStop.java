package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hood;

public class HoodStop extends InstantCommand {
  Hood mAngledShooter = Hood.getInstance();

  public HoodStop() {
    addRequirements(mAngledShooter);
  }

  @Override
  public void initialize() {
    // System.out.println("&&&&&&&&&&&&&&&&&&_ANGLED_SHOOTER_0FF_&&&&&&&&&&&&&&&&&&");
    // mAngledShooter.pctCtrl_set(0.0);
    mAngledShooter.pctCtrl_stopHoodMotor();
  }
}
