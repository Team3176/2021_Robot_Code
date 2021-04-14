package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.AngledShooterConstants;
import frc.robot.subsystems.AngledShooter;

public class AngledShooterDown extends InstantCommand {

  private AngledShooter m_AngledShooter = AngledShooter.getInstance();

  public AngledShooterDown() {
    addRequirements(m_AngledShooter);
  }

  @Override
  public void initialize() {
    // System.out.println("AngledShooterDown.initialize executed. ############################################################");
    /* Gets the current encoder position and see where it should go */
    
    /*
    double temp = m_AngledShooter.getEncoderPosition();
    if(temp <= AngledShooterConstants.pos[1] + 100) {
     m_AngledShooter.setPosition(AngledShooterConstants.pos[0]);
    } else if(temp <= AngledShooterConstants.pos[2] + 100 && temp >= AngledShooterConstants.pos[0] - 100) {
       m_AngledShooter.setPosition(AngledShooterConstants.pos[1]);
    } else if(temp <= AngledShooterConstants.pos[3] + 100 && temp >= AngledShooterConstants.pos[1] - 100) {
     m_AngledShooter.setPosition(AngledShooterConstants.pos[2]);
    } else if(temp <= AngledShooterConstants.pos[4] + 100 && temp >= AngledShooterConstants.pos[2] - 100) {
     m_AngledShooter.setPosition(AngledShooterConstants.pos[3]);
     */

    // m_AngledShooter.goDownToNextHoodPosition_Tic();

    //m_AngledShooter.pctCtrl_lowerHoodPosition();
    m_AngledShooter.pctCtrl_set(-0.1);
  }
}