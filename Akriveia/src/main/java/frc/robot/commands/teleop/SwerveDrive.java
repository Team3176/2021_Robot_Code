package frc.robot.commands.teleop;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DrivetrainConstants;
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

  public SwerveDrive( DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand) {
 //                     BooleanSupplier isFieldCentric, BooleanSupplier isRobotCentric ) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;
    //this.isFieldCentric = isFieldCentric;
    //this.isRobotCentric = isRobotCentric;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DRIVE);
    drivetrain.setSpinLock(false);
  }

  @Override
  public void execute() {

    if(drivetrain.getCurrentCoordType() == coordType.FIELD_CENTRIC) {
      //drivetrain.setCoordType(coordType.FIELD_CENTRIC);
      drivetrain.setFieldCentricOffset();
    }
    //if(isRobotCentric.getAsBoolean()) {
    //  drivetrain.setCoordType(coordType.ROBOT_CENTRIC);
    //}
    drivetrain.drive(forwardCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND, strafeCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND, spinCommand.getAsDouble()*10);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}