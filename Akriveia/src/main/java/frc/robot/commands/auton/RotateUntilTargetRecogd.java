// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.sql.Driver;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.coordType;
import frc.robot.subsystems.Vision;


/**
 * RotateUntilTargetRecogd
 */
public class RotateUntilTargetRecogd extends CommandBase {

  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private Vision m_Vision = Vision.getInstance();
  private boolean tv;

  public RotateUntilTargetRecogd() {
    addRequirements(m_Drivetrain);
  }

  @Override
  public void initialize() {
    m_Drivetrain.setCoordType(coordType.ROBOT_CENTRIC);
    m_Vision.turnLEDsOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.tv =  m_Vision.getTv();
    //new AutonRotate(.1, tx);
    m_Drivetrain.drive(0, 0, .5);
    // SmartDashboard.putBoolean("AutonAlign.tv", tv);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.drive(0,0,0);
      m_Vision.turnLEDsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tv;
  }
}
