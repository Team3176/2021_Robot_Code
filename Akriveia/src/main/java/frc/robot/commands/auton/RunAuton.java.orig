// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AngledShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAuton extends SequentialCommandGroup {
  /** Creates a new RunAuton. */
  public RunAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new AutonDrive(1,0,0,4)//,

      //new AutonDrive(0,.25,0,1)
    
      // new AutonRotate(.01, 90)

      new TrapezoidDrive(9.5, -6.1)


    );
  }
}
