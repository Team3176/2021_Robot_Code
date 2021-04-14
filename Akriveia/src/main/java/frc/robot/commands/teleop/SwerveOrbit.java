package frc.robot.commands.teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.coordType;
import frc.robot.subsystems.Drivetrain.driveMode;
public class SwerveOrbit extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();

  private DoubleSupplier orbitSpeed;
  private DoubleSupplier pov;

  private double orbitEtherRadius = 30.0; // inches

  private boolean wasFieldCentric;

  public SwerveOrbit(DoubleSupplier orbitSpeed, DoubleSupplier pov) {
    this.orbitSpeed = orbitSpeed;
    this.pov = pov;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    if(drivetrain.getCurrentCoordType() == coordType.FIELD_CENTRIC) {
      wasFieldCentric = true;
    } else {
      wasFieldCentric = false;
    }
    drivetrain.setDriveMode(driveMode.ORBIT);
    drivetrain.setCoordType(coordType.ROBOT_CENTRIC);
  }

  @Override
  public void execute() {
    if(pov.getAsDouble() == 45.0 || pov.getAsDouble() == 90.0 || pov.getAsDouble() == 135.0) { // If on right side
      // Orbit Clockwise
      drivetrain.drive(orbitSpeed.getAsDouble(), 0.0, orbitSpeed.getAsDouble() / orbitEtherRadius /* inches */);
    } 
    else if (pov.getAsDouble() == 225.0 || pov.getAsDouble() == 270.0 || pov.getAsDouble() == 315.0) { // If on left side
      // Orbit Counter-Clockwise
      drivetrain.drive(orbitSpeed.getAsDouble(), 0.0, -orbitSpeed.getAsDouble() / orbitEtherRadius /* inches */);
    }
    
    
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {
    if(wasFieldCentric) {
      drivetrain.setCoordType(coordType.FIELD_CENTRIC);
    } else {
      drivetrain.setCoordType(coordType.ROBOT_CENTRIC);
    }
  }
}
