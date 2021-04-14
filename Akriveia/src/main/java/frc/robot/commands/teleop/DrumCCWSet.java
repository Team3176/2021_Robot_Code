package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drum;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrumCCWSet extends CommandBase {
  Drum m_Drum = Drum.getInstance();
  public DrumCCWSet() {
    addRequirements(m_Drum);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("CCWPCT", 0); 
    m_Drum.CounterClockwise(0);
  }

  @Override
  public void execute() {
    double PCT = SmartDashboard.getNumber("CCWPCT", 0);
    m_Drum.CounterClockwise(PCT);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (SmartDashboard.getNumber("CCWPCT", 0) == -999) {  //TODO: check/test to make sure this conditional evaluation with SmartDashboard getNumber will work properly
      return true;
    } else {
      return false;
    }
  }
}
