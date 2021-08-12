// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Transfer;

public class IrCounter extends CommandBase {
  Transfer m_Transfer = Transfer.getInstance();
  int count;
  boolean lastState;
  int endCounts;

  public IrCounter(int maxCounts) {
    endCounts = maxCounts;
    addRequirements(m_Transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Transfer.getIrSensor() && lastState == false) {
      count++;
      lastState = true;
      System.out.println("----IR__COUNT__" + count + "----");
    }
    else if(!m_Transfer.getIrSensor()) {
      lastState = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("----IrCounter__END----");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(count == endCounts) {return true;}
    return false;
  }
}
