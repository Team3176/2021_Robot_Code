package frc.robot.commands.auton;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.coordType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Slalom extends CommandBase {
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private double startTime;
    double runTimeInput;
    double time1 = 1.15;
    double time2 = 1.43;
    double time3 = 3.5; // It's about that but still need to est

    public Slalom() {
        addRequirements(drivetrain);
        // runTimeInput = SmartDashboard.getNumber("runTime", 0.5);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        drivetrain.setCoordType(coordType.FIELD_CENTRIC);
        drivetrain.resetGyro();
    }

    @Override
    public void execute() {
        if((startTime + time1) > Timer.getFPGATimestamp()) {
            drivetrain.drive(0.5, 0.0, 0.0);
        } else if((startTime + time1 + time2) > Timer.getFPGATimestamp()) {
            drivetrain.drive(0.0, -0.5, 0.0);
        // } else if((startTime + time1 + time2 + SmartDashboard.getNumber("runTime", runTimeInput)) > Timer.getFPGATimestamp()) {
            // drivetrain.drive(0.5, 0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        // return (startTime + time1 + time2 + SmartDashboard.getNumber("runTime", runTimeInput)) < Timer.getFPGATimestamp();
        return true;
    }

    @Override
    public void end(boolean interrupted) { 
        drivetrain.drive(0.0, 0.0, 0.0);
    }
}