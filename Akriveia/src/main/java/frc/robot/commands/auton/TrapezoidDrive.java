// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.coordType;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class TrapezoidDrive extends CommandBase {
  /** Creates a new TrapezoidDrive. */
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private Timer timer = new Timer();
  private TrapezoidProfile profile;
  private double theta;
  private double distanceX;
  private double distanceY;

  public TrapezoidDrive(double distanceX, double distanceY) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
    this.distanceX = distanceX;
    this.distanceY = distanceY;
    double hyp = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
    this.theta = Math.atan(distanceY / distanceX);
    profile = new TrapezoidProfile
    (new TrapezoidProfile.Constraints(5, 2), new TrapezoidProfile.State(hyp, 0), new TrapezoidProfile.State(0, 0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Drivetrain.setCoordType(coordType.FIELD_CENTRIC);
    m_Drivetrain.setSpinLockAngle();
    m_Drivetrain.setSpinLock(true);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrapezoidProfile.State setPoint = profile.calculate(timer.get());
    double velocityX = setPoint.velocity * Math.cos(theta);
    double velocityY = setPoint.velocity * Math.sin(theta);
    m_Drivetrain.drive(Math.copySign(velocityX, distanceX), Math.copySign(velocityY, distanceY), 0);

    // System.out.println("X: " + distanceX + ", Y: " + distanceY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.setSpinLock(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return profile.isFinished(timer.get());
  }
}
