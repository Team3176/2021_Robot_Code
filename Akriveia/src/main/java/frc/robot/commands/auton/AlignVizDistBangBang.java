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
import frc.robot.subsystems.Drivetrain.coordType;
import frc.robot.subsystems.Vision;
import frc.robot.constants.VisionConstants;

/**
 * AlignVizDistBangBang 
 */
public class AlignVizDistBangBang extends CommandBase {

  private Drivetrain m_drivetrain = Drivetrain.getInstance();
  private Vision m_Vision = Vision.getInstance();
  private double currentDistToTarget_meters, deltaX_meters;  //in meters
  private double acceptableDeltaXError_meters, acceptableDeltaXError_feet; 
  private double desiredDistToTarget_meters, desiredDistToTargetLowerLimit_meters, desiredDistToTargetUpperLimit_meters;
  private double forwardDriveCorrection_meters, forwardDriveCorrection_feet;

  /**
   * 
   * @param dist: desired distance to Target (in feet)
   */
  public AlignVizDistBangBang(double dist) {
    addRequirements(m_drivetrain);
    desiredDistToTarget_meters = dist / VisionConstants.FEET2METER;  // convert dist from feet to meters and store as desiredDistToTarget 
  }

  @Override
  public void initialize() {
    m_drivetrain.setCoordType(coordType.ROBOT_CENTRIC);
    m_Vision.turnLEDsOn();
    acceptableDeltaXError_feet = 4;
    this.acceptableDeltaXError_meters= acceptableDeltaXError_feet / VisionConstants.FEET2METER;
    this.desiredDistToTargetLowerLimit_meters = desiredDistToTarget_meters - acceptableDeltaXError_meters;
    this.desiredDistToTargetUpperLimit_meters = desiredDistToTarget_meters + acceptableDeltaXError_meters;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.deltaX_meters =  m_Vision.getDeltaX();
    currentDistToTarget_meters = deltaX_meters;
    if (currentDistToTarget_meters >= desiredDistToTargetLowerLimit_meters) {
      forwardDriveCorrection_meters = currentDistToTarget_meters - desiredDistToTarget_meters;
      forwardDriveCorrection_feet = forwardDriveCorrection_meters / VisionConstants.FEET2METER;
    } else if (currentDistToTarget_meters <= desiredDistToTargetUpperLimit_meters) {
      forwardDriveCorrection_meters = -(desiredDistToTarget_meters - currentDistToTarget_meters);
      forwardDriveCorrection_feet = forwardDriveCorrection_meters / VisionConstants.FEET2METER;
    }

    m_drivetrain.drive(forwardDriveCorrection_feet, 0, 0);
    // SmartDashboard.putNumber("AlignVizDistBangBang.forwardDriveCorrection_feet", forwardDriveCorrection_feet);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0,0,0);
    m_Vision.turnLEDsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if ((currentDistToTarget_meters >= desiredDistToTargetLowerLimit_meters) && (currentDistToTarget_meters <= desiredDistToTargetUpperLimit_meters)) {
      return true;
    } else {
      return false;
    }
  }
}
