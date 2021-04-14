package frc.robot.commands.teleop;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.coordType;
import frc.robot.subsystems.Drivetrain.driveMode;

public class SwerveDrive extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private DoubleSupplier forwardCommand;
  private DoubleSupplier strafeCommand;
  private DoubleSupplier spinCommand;

  private BooleanSupplier isFieldCentric;
  private BooleanSupplier isRobotCentric;
  private BooleanSupplier isBackRobotCentric;

  public SwerveDrive( DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand,
                      BooleanSupplier isFieldCentric, BooleanSupplier isRobotCentric, BooleanSupplier isBackRobotCentric) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;
    this.isFieldCentric = isFieldCentric;
    this.isRobotCentric = isRobotCentric;
    this.isBackRobotCentric = isBackRobotCentric;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DRIVE);
  }

  @Override
  public void execute() {

    if(isFieldCentric.getAsBoolean()) {
      drivetrain.setCoordType(coordType.FIELD_CENTRIC);
    }
    if(isRobotCentric.getAsBoolean()) {
      drivetrain.setCoordType(coordType.ROBOT_CENTRIC);
    }
    if(isBackRobotCentric.getAsBoolean()) {
      drivetrain.setCoordType(coordType.BACK_ROBOT_CENTRIC);
    }
    drivetrain.drive(forwardCommand.getAsDouble(), strafeCommand.getAsDouble(), spinCommand.getAsDouble());
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}