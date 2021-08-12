package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Drum;


public class IntakeHarvestStart extends InstantCommand {
  Drum m_Drum = Drum.getInstance();


  public IntakeHarvestStart() {
    addRequirements(m_Drum);

  }

  @Override
  public void initialize() {
    // System.out.println("IntakeHarvest.initialize executed. ############################################################");
    m_Drum.CounterClockwise();
    // mIntake.deployIntake();
    // System.out.println("HARVEST");
  }
}
