
package frc.robot.commands.teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.VisionClient;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.driveMode;

import frc.robot.util.PIDLoop;

/* 
My hope for vision is that we'll be able to translate however we want but control of spin
will be handled by code that will make our robot constantly point at the target. If all we works
well, we could drive across the field while staying locked onto the target with automated spin in code
*/

public class SwerveVision extends CommandBase {
  private VisionClient visionClient = VisionClient.getInstance();
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private DoubleSupplier forwardCommand;
  private DoubleSupplier strafeCommand;

  private PIDLoop spinPID;
  private double spinOutput;

  public SwerveVision(DoubleSupplier forwardCommand, DoubleSupplier strafeCommand) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    addRequirements(drivetrain);

    spinPID = new PIDLoop(0.3, 0.0, 0.0, 0.0);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.VISION);
  }

  @Override
  public void execute() {
    spinOutput = spinPID.returnOutput(drivetrain.getGyroAngle(), drivetrain.getGyroAngle() + visionClient.getBallDegrees());
    drivetrain.drive(forwardCommand.getAsDouble(), strafeCommand.getAsDouble(), spinOutput);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}