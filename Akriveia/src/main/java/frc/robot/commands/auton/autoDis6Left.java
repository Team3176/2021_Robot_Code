// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drum;
import edu.wpi.first.wpilibj.Timer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoDis6Left extends SequentialCommandGroup {
  
  private double startTime;

  public autoDis6Left() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    startTime = Timer.getFPGATimestamp();
    addCommands(
      // Robot starts facing the target directly in front of it
      // Also we know magnitudes but not signs for TrapezoidDrive

      //We shoot
      new TrapezoidDrive(-1, -6), // Back 1 foot and left 6 feet. 
      //new DelayCommand(10 - (Timer.getFPGATimestamp() - startTime)),
      new ShootVisionSetUp()
      //new TrapezoidDrive(0, -16) // Backwards 16 feet
    );
  }
}
