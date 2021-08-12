// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.*;

public class AutonSelector extends CommandBase{
  private Vision m_Vision = Vision.getInstance();
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private CommandScheduler commandScheduler = CommandScheduler.getInstance();

  private double angle;

  public AutonSelector(){
    addRequirements(m_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    m_Vision.controlLoopBallRecog();
    angle = (m_Vision.hasTarget == 1) ? m_Vision.ballAngle * -VisionConstants.DEG2RAD : -999;
  }

  /**
   * KEY:
   * A - red: about 0
   * A - blue: about -.3805063771 radians
   * B - red: about .463647609 radians
   * B - blue: about -.197395598 radians
   * 
   * distance between left edge and Ab: 17.2 degrees
   * distance between Ab and Bb: 10.5 degrees
   * distance between Bb and Ar: 11.3 degrees
   * distance between Ar and Br: 26.6 degrees
   * distance between Br and right edge: 1.1 degrees
   */
  @Override
  public void execute(){
    commandScheduler.schedule(new IntakeDropViaSpinning());

    // SmartDashboard.putNumber("Received Angle (rad)", angle);
    if(angle == -999){
      // print out an error message
      // SmartDashboard.putString("Path", "UNKNOWN");
    } else if(angle < -.29){
      // A - blue
      // SmartDashboard.putString("Path", "A - blue");
      m_Drivetrain.drive(Math.cos(-.3805063771), Math.sin(-.3805063771), 0);
    } else if(angle < -.08){
      // B - blue
      // SmartDashboard.putString("Path", "B - blue");
      m_Drivetrain.drive(Math.cos(-.197395598), Math.sin(-.197395598), 0);
    } else if(angle < .15){
      // A - red
      // SmartDashboard.putString("Path", "A - red");
      m_Drivetrain.drive(Math.cos(0), Math.sin(0), 0);
    } else if(angle < .52){
      // B - red
      // SmartDashboard.putString("Path", "B - red");
      m_Drivetrain.drive(Math.cos(.463647609), Math.sin(.463647409), 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){}

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return true;
  }
}
