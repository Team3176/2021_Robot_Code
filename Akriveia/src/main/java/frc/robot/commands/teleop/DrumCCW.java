package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drum;

public class DrumCCW extends CommandBase {
  
  Drum m_Drum = Drum.getInstance();

  public DrumCCW() {
    addRequirements(m_Drum);
  }

  @Override
  public void initialize() {
    // System.out.println("DrumCCW.initialize executed. ############################################################");

  }

  @Override
  public void execute() {
    //if(m_Drum.getOneTimeIRSwitch()){
    //if(m_Drum.getDrumStopMotorFlag()){
    m_Drum.CounterClockwise();
  //}
  //else{
  //  m_Drum.CounterClockwise();
  //}
  
//}
  }

  @Override
  public boolean isFinished() {
    if(m_Drum.getDrumStopMotorFlag()){
    return true; 
  }
      return false;
    
  }

  @Override
  public void end(boolean interrupted) {  

  }
  
}
