// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

/**
 * @author Jared Brown
 */
public class DelayCommand extends CommandBase {
  /** Creates a new DelayCommand. */
  private Timer timer;
  private double delayTime;
  private double startTime;

  /**
  * For use in auton, to wait a time in seconds between sequenced commands.
  @param delayTime Time to wait in seconds
  */
  public DelayCommand(double delayTime) {
    this.delayTime = delayTime;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();
    startTime = timer.getFPGATimestamp();
    System.out.println("-----TIMER STARTED------");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    System.out.println("----TIMER ENDED----");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime >= delayTime);
  }
}
