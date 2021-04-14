// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.constants.IntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeReverse extends InstantCommand {
  Intake m_Intake = Intake.getInstance();
  public IntakeReverse() {
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("IntakeReverse.initialize executed. ############################################################");
    /* Gets the current intake speed and sets it clockwise */

    if (m_Intake.getIntakeMotorSpeed() == IntakeConstants.INTAKE_PERCENT) {
      m_Intake.setPercentControl(-IntakeConstants.INTAKE_PERCENT);
      // System.out.println("The intake is set to: " + -IntakeConstants.INTAKE_PERCENT);
    } else if(m_Intake.getIntakeMotorSpeed() == -IntakeConstants.INTAKE_PERCENT) {
      m_Intake.setPercentControl(IntakeConstants.INTAKE_PERCENT);
      // System.out.println("The intake is set to: " + IntakeConstants.INTAKE_PERCENT);
    } else {
      m_Intake.setPercentControl(0);
      // System.out.println("The intake is set to 0");
    }
  }
}
