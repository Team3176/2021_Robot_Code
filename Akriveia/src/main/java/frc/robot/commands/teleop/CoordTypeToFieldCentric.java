package frc.robot.commands.teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.coordType;

public class CoordTypeToFieldCentric extends InstantCommand {
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();


  public CoordTypeToFieldCentric() {
    addRequirements(m_Drivetrain);
  }

  @Override
  public void initialize() {
    m_Drivetrain.setCoordType(coordType.FIELD_CENTRIC);
  }

}
