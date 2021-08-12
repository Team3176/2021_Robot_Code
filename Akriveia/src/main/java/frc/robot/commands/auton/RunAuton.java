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
public class RunAuton extends SequentialCommandGroup {
  /** Creates a new RunAuton. */
  public RunAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new AutonDrive(1,0,0,4)//,
      // new AlignVizYawBangBang()
      new AlignVizYawPLoop()
      //new ShootVisionSetUp()
      // new TrapezoidDrive(-5, 0)
      //new AutonDrive(0,.25,0,1)
    
      //  new AutonRotate(.1, 720),
        // new DelayCommand(1)
      //  new AutonRotate(.1, -720)
  
    // new TrapezoidDrive(10, 0),
    // new AlignVizDistBangBang(15)
      // new TrapezoidDrive(-13, 0)

      // new TrapezoidDrive(-7, -10)
      // Flywheel spin up
      // wait for 10 seconds?
      // new TrapezoidDrive(22, -5)


      // GYRO NOTE:
      // gyro off by -117.5 @ 8:02p, -117.86 @ 8:03p -- not touched during that time
      // continuing to drift more negative while not moving



    );
  }
}
