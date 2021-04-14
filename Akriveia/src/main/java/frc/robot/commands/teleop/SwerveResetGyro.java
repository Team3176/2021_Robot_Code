package frc.robot.commands.teleop;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import javax.net.ssl.TrustManagerFactorySpi;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

/**
 * Makes the gyro's "zero point" its current position, for recallibration.
 */
public class SwerveResetGyro extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private boolean isreset;

  public SwerveResetGyro() {
    addRequirements(drivetrain);
    isreset = false;
  }

  @Override
  public void initialize() {  
  }

  @Override
  public void execute() {
    drivetrain.resetGyro();
    isreset = true;
  }

  @Override
  public boolean isFinished() { 
    if (isreset) {
      return true;
    } else {
      return false;
    } 
  }

  @Override
  public void end(boolean interrupted) { 
  }

}