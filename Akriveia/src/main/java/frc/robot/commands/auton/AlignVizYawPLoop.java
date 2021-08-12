// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.sql.Driver;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.AutonRotate;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Drivetrain.coordType;
import frc.robot.subsystems.Vision;


/**
 * AutoAlign: A simplistic command class to retrieve the x-angle (tx) formed
 * by the LL crosshairs, the lens, and the recognized target.  It (tx) is then
 * used to call AutoRotate(tx) to rotate the bot until the angle is within the range 
 * formed by upperTxLimit and lowerTxLimit
 */
public class AlignVizYawPLoop extends SequentialCommandGroup {

  private Drivetrain m_drivetrain = Drivetrain.getInstance();
  private Vision m_Vision = Vision.getInstance();
  private Transfer m_Transfer = Transfer.getInstance();
  private double tx, yawError, steerCorrection;
  private double upperTxLimit, lowerTxLimit;
  private double kP, minCommand;

  /** Creates a new AutonAlign. */
  public AlignVizYawPLoop() {
    addRequirements(m_drivetrain);
    addRequirements(m_Transfer);
  }

  @Override
  public void initialize() {
    // m_drivetrain.setCoordType(coordType.ROBOT_CENTRIC);
    m_drivetrain.setCoordType(coordType.FIELD_CENTRIC);
    //m_Vision.setPipeline(1);
    //m_Vision.turnLEDsOn();
    this.kP = -0.01;
    this.minCommand = 0.001;
    this.upperTxLimit = 2;
    this.lowerTxLimit = -2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.tx =  m_Vision.getTx();
    this.yawError = -tx;
    //new AutonRotate(.1, tx);
    if (tx > upperTxLimit) {
      steerCorrection =  kP * yawError - minCommand;
    } else if ( tx < lowerTxLimit ) {
      steerCorrection = kP * yawError + minCommand;
    } 
    //m_drivetrain.drive(0, 0, Math.copySign(.1, yawError));
    m_drivetrain.drive(0, 0, -steerCorrection);
    // SmartDashboard.putNumber("AlignViaYawPLoop.tx", tx);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_drivetrain.drive(0,0,0);
      //m_Vision.setPipeline(3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this.yawError >= lowerTxLimit && this.yawError <= upperTxLimit) {
      return true;
    }
    return false;
  }
}
