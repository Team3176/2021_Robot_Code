// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.VisionClient;
import frc.robot.subsystems.AngledShooter;
import frc.robot.constants.AngledShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Flywheel;

/**
 * Attempts to use VisionClient data to set the AngledShooter and Flywheel to the correct angle and velocity,
 * respectively, to make a shot into the target.
 * @author Jared Brown
 * @see VisionClient
 */
public class TargetVision extends CommandBase {
  /** Creates a new TargetVision. */
  AngledShooter m_AngledShooter = AngledShooter.getInstance();
  VisionClient m_VisionClient = VisionClient.getInstance();
  Flywheel m_Flywheel = Flywheel.getInstance();
  private double wantedAngleTicks;
  private double wantedVelocityTicks; // Ticks per 100 ms

  public TargetVision() {
    addRequirements(m_AngledShooter);
    addRequirements(m_Flywheel);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double[] resultArray = m_VisionClient.targetRecogControlLoop();
    if (resultArray != null) {
      wantedAngleTicks = resultArray[1] * (180 / Math.PI) * (AngledShooterConstants.TICS_EQUAL_TO_5DEGREES / 5);
      m_AngledShooter.pidPosCtrl_setPosition(wantedAngleTicks); // range constraints checked in AngledShooter by this method

      // m/s to rad/s (using alpha = r * omega) to rev/s to rev/min (rpm)
      wantedVelocityTicks = (resultArray[0] / VisionConstants.FLYWHEEL_RADIUS / (2 * Math.PI) * 60);
      m_Flywheel.spinVelocityPIDFPart2(wantedVelocityTicks);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
