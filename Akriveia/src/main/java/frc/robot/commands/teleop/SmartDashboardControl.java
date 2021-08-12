// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Flywheel;

public class SmartDashboardControl extends CommandBase {
  public Flywheel fly = Flywheel.getInstance();
  public Drum drum = Drum.getInstance();

  public SmartDashboardControl() {
    addRequirements(fly);
    addRequirements(drum);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("fly pct", 0);
    // SmartDashboard.putNumber("drum pct", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double fPCT = SmartDashboard.getNumber("fly pct", 0);
    // double dPCT = SmartDashboard.getNumber("drum pct", 0);

    // fly.spinVelocityOutputPercent(fPCT);
    // drum.pctCtrl_set(dPCT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    fly.spinVelocityOutputPercent(0);
    drum.pctCtrl_set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
